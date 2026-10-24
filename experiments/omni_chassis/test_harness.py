"""Decisive mathematical checks, independent geometry/LP oracles and regeneration."""

from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from numpy.testing import assert_allclose
from scipy.optimize import linprog

from model import OmniModel
from run import rigid_body_drive_speeds, run_experiment


ROOT = Path(__file__).resolve().parent


def fixture():
    return json.loads((ROOT / "configs/synthetic_symmetric4.json").read_text())


class HarnessTest(unittest.TestCase):
    def test_analytic_octahedron_and_unsafe_box(self):
        m = OmniModel(fixture())
        c = 40 * .05 * np.sqrt(2)
        expected = np.vstack((np.diag([c, c, 2*c]), -np.diag([c, c, 2*c])))
        self.assertEqual(len(m.vertices), 6)
        for vertex in expected:
            self.assertLess(np.linalg.norm(m.vertices - vertex, axis=1).min(), 1e-10)
        self.assertAlmostEqual(m.volume, 4/3 * c**3 / .5, places=10)
        self.assertAlmostEqual(m.axis_box.volume/m.volume, 6., places=10)
        self.assertAlmostEqual(m.inner_box.volume/m.volume, 2/9, places=10)
        self.assertTrue(np.all(m.contains(m.inner_box.corners)))
        self.assertTrue(np.all(~m.contains(m.axis_box.corners)))
        # An independent closed-form predicate over arbitrary simultaneous commands.
        v = np.random.default_rng(9).uniform(-5, 5, (1000, 3))
        expected_mask = abs(v[:, 0]) + abs(v[:, 1]) + .5*abs(v[:, 2]) <= c
        np.testing.assert_array_equal(m.contains(v), expected_mask)

    def test_point_velocity_and_lp_support_for_all_configs(self):
        rng = np.random.default_rng(36)
        for path in sorted((ROOT / "configs").glob("synthetic_*.json")):
            with self.subTest(config=path.name):
                m = OmniModel.load(path)
                v = rng.normal(size=(100, 3)) * m.scale
                assert_allclose(m.inverse(v), rigid_body_drive_speeds(m, v), atol=1e-11)
                assert_allclose(m.forward(m.inverse(v)), v, atol=1e-11)
                # LP is independent of triple-plane vertex enumeration. Includes
                # six axes plus arbitrary support directions to catch missing vertices.
                for direction in np.vstack((np.eye(3), -np.eye(3), rng.normal(size=(20, 3)))):
                    result = linprog(-direction, A_ub=m.A, b_ub=m.b,
                                     bounds=[(None, None)] * 3, method="highs")
                    self.assertTrue(result.success, result.message)
                    self.assertAlmostEqual((m.vertices @ direction).max(), -result.fun, places=8)
                r = run_experiment(m, 1000, [17, 37], boundary_samples=100)
                self.assertTrue(r["passed"], r["gates"])

    def test_parameter_changes_and_wheel_permutation(self):
        base = fixture()
        m = OmniModel(base)
        for field, value, row_factor in (("radius_m", .1, .5),
                                          ("drive_per_wheel_ratio", 7., 7.),
                                          ("drive_sign", -1, -1.)):
            changed = deepcopy(base)
            changed["wheels"][0][field] = value
            n = OmniModel(changed)
            assert_allclose(n.J[0], m.J[0] * row_factor)
            assert_allclose(n.J[1:], m.J[1:])
        changed = deepcopy(base)
        changed["wheels"][0]["position_xy_m"][0] += .1
        n = OmniModel(changed)
        self.assertNotAlmostEqual(n.J[0, 2], m.J[0, 2])
        changed = deepcopy(base)
        changed["wheels"][0]["rolling_direction_xy"] = [.8, -.6]
        self.assertFalse(np.allclose(OmniModel(changed).J[0], m.J[0]))
        changed = deepcopy(base)
        for wheel in changed["wheels"]:
            wheel["speed_bounds_drive_rad_s"] = [-20, 20]
        self.assertAlmostEqual(OmniModel(changed).volume, m.volume / 8, places=10)
        perm = [2, 0, 3, 1]
        changed = deepcopy(base)
        changed["wheels"] = [base["wheels"][i] for i in perm]
        n = OmniModel(changed)
        assert_allclose(n.J, m.J[perm])
        assert_allclose(n.K[:, np.argsort(perm)], m.K, atol=1e-12)
        self.assertAlmostEqual(n.volume, m.volume, places=10)

    def test_random_layouts_and_redundant_feedback(self):
        rng = np.random.default_rng(20261024)
        for count in (3, 4, 6, 8):
            for _ in range(4):
                config = fixture()
                config["wheels"] = []
                for i in range(count):
                    angle = rng.uniform(-np.pi, np.pi)
                    w = deepcopy(fixture()["wheels"][0])
                    w.update(name=str(i), position_xy_m=rng.uniform(-.4, .4, 2).tolist(),
                             rolling_direction_xy=[float(np.cos(angle)), float(np.sin(angle))],
                             radius_m=float(rng.uniform(.03, .1)),
                             drive_per_wheel_ratio=float(rng.uniform(1, 20)),
                             drive_sign=int(rng.choice([-1, 1])),
                             speed_bounds_drive_rad_s=[-float(rng.uniform(30, 300)),
                                                       float(rng.uniform(30, 300))])
                    config["wheels"].append(w)
                m = OmniModel(config)
                v = rng.normal(size=(200, 3))
                assert_allclose(m.inverse(v), rigid_body_drive_speeds(m, v), atol=1e-10)
                assert_allclose(m.forward(m.inverse(v)), v, atol=1e-10)
                self.assertTrue(np.all(m.contains(m.radial_clip(v))))
        m = OmniModel(fixture())
        omega = np.array([40., 40., 40., -40.])
        self.assertGreater(np.linalg.norm(m.residual(omega)), 1.)
        # A bounded but inconsistent wheel vector does not imply its LS chassis
        # estimate is wheel-feasible. Independent wheel clipping is not our API.
        self.assertFalse(m.contains(m.forward(omega)))
        assert_allclose(m.J.T @ m.residual(omega), 0., atol=1e-10)

    def test_affine_solver_pullback_and_batch_shapes(self):
        m = OmniModel(fixture())
        rng = np.random.default_rng(71)
        mapping = rng.normal(size=(3, 5))
        offset = np.array([.2, -.3, .1])
        C, d = m.linear_constraints(mapping, offset)
        z = rng.normal(size=(10, 5))
        assert_allclose(d - z @ C.T, m.slack(z @ mapping.T + offset), atol=1e-12)
        # Also describes one velocity step under a body-coordinate acceleration
        # abstraction; no acceleration/dynamics claim is inferred from it.
        C, d = m.linear_constraints(.02*np.eye(3), offset)
        assert_allclose(d - C @ np.ones(3), m.slack(offset + .02))
        v = rng.normal(size=(2, 4, 3))
        assert_allclose(m.forward(m.inverse(v)), v, atol=1e-12)
        self.assertTrue(np.all(m.contains(m.radial_clip(v))))
        assert_allclose(m.radial_clip(np.zeros(3)), np.zeros(3))

    def test_unknown_or_degenerate_geometry_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "missing"):
            OmniModel.load(ROOT / "configs/mcu_pending.template.json")
        bad = fixture()
        for w in bad["wheels"]:
            w["rolling_direction_xy"] = [1., 0.]
        with self.assertRaisesRegex(ValueError, "rank-deficient"):
            OmniModel(bad)
        for field, value in (("radius_m", 0), ("drive_sign", 0),
                             ("rolling_direction_xy", [1, 1]),
                             ("speed_bounds_drive_rad_s", [0, 40]),
                             ("drive_per_wheel_ratio", float("nan"))):
            with self.subTest(field=field), self.assertRaises(ValueError):
                bad = fixture()
                bad["wheels"][0][field] = value
                OmniModel(bad)

    def test_cli_config_regeneration_and_incomplete_template(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "generated"
            cmd = [sys.executable, str(ROOT / "run.py"), "--samples-per-seed", "200",
                   "--seeds", "11", "--output", str(output), "--config"]
            result = subprocess.run(cmd + [str(ROOT / "configs/mcu_pending.template.json")],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertFalse(output.exists())
            config = fixture()
            config["name"] = "replacement_mcu_fixture"
            config["wheels"][0]["radius_m"] = .071
            path = Path(directory) / "replacement.json"
            path.write_text(json.dumps(config))
            subprocess.run(cmd + [str(path)], capture_output=True, text=True, check=True)
            saved = json.loads((output / "replacement/model.json").read_text())
            assert_allclose(saved["inverse_J_drive"], OmniModel(config).J)
            self.assertFalse(saved["hardware_validated"])
            self.assertTrue(json.loads((output / "results.json").read_text())["experiments"][0]["passed"])


if __name__ == "__main__":
    unittest.main()
