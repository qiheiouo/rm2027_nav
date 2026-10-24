"""Mathematical oracles and interface boundaries for stage-two research."""

from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from numpy.testing import assert_allclose

from model import OmniModel, NumericalGeometryError
from optimization import CommandProjector, QuadraticProjector, maximum_box, tinympc_candidate
from stage2 import reference_projection, validation
from uncertainty import row_derivatives, perturb_config, utilization_upper_bound, robust_ray_research


ROOT = Path(__file__).resolve().parent


def fixture():
    return json.loads((ROOT/"configs/synthetic_symmetric4.json").read_text())


class StageTwoTest(unittest.TestCase):
    def test_maximum_box_analytic_volume_and_global_certificate(self):
        m = OmniModel(fixture())
        for policy in ("origin", "free", "contains_zero"):
            result = maximum_box(m, policy)
            self.assertAlmostEqual(result["box"].volume/m.volume, 2/9, places=8)
            self.assertLess(result["log_volume_gap_bound"], 1e-7)
            self.assertTrue(m.contains(result["box"].corners).all())
        m = OmniModel.load(ROOT/"configs/synthetic_asymmetric4.json")
        zero = maximum_box(m)["box"]
        origin = maximum_box(m, "origin")["box"]
        self.assertGreater(zero.volume, m.inner_box.volume)
        self.assertGreater(zero.volume, origin.volume)
        self.assertTrue(zero.contains(np.zeros(3)))

    def test_quadratic_projection_against_independent_solver_and_metric_change(self):
        m = OmniModel.load(ROOT/"configs/synthetic_asymmetric4.json")
        rng = np.random.default_rng(12)
        W = np.array([[2., .4, .1], [.4, 1., -.2], [.1, -.2, 3.]])
        q = QuadraticProjector(m, W)
        u = rng.normal(size=(30, 3))*m.scale*2
        v = q.project(u)
        self.assertTrue(m.contains(v, tolerance=1e-8).all())
        self.assertLess(q.optimality_residual(u, v), 1e-8)
        assert_allclose(q.project(v), v, atol=1e-9)
        for source, projected in zip(u[:10], v[:10]):
            oracle = reference_projection(m, source, W)
            assert_allclose(projected, oracle, atol=2e-6)
        # A large yaw priority has a concrete different optimizer, not an alias.
        m = OmniModel(fixture())
        source = np.array([2., 2., 4.])
        plain = QuadraticProjector(m).project(source)
        priority = QuadraticProjector(m, np.diag([1., 1., 100.])).project(source)
        self.assertLess(abs(priority[2]-source[2]), abs(plain[2]-source[2]))

    def test_free_maximum_box_can_exclude_stop(self):
        config = json.loads((ROOT/"configs/synthetic_three.json").read_text())
        for w in config["wheels"]:
            w["drive_sign"] = 1
            w["speed_bounds_drive_rad_s"] = [-.01, 40.]
        m = OmniModel(config)
        free = maximum_box(m, "free")
        self.assertFalse(free["contains_zero"])
        safe = maximum_box(m)
        self.assertTrue(safe["contains_zero"])
        self.assertGreater(free["box"].volume, safe["box"].volume)
        with self.assertRaisesRegex(ValueError, "zero command"):
            CommandProjector(m, "maximum_box", box=free["box"])

    def test_uniform_radius_or_limit_error_has_exact_cubic_volume_effect(self):
        config = fixture()
        m = OmniModel(config)
        edge = m.vertices
        for field in ("radius_m", "speed_bounds_drive_rad_s"):
            changed = deepcopy(config)
            for w in changed["wheels"]:
                value = np.asarray(w[field])*.99
                w[field] = float(value) if value.ndim == 0 else value.tolist()
            actual = OmniModel(changed)
            self.assertAlmostEqual(actual.volume/m.volume, .99**3, places=10)
            util = np.maximum(actual.inverse(edge)/actual.upper, actual.inverse(edge)/actual.lower).max(axis=1)
            assert_allclose(util, 1/.99, atol=1e-12)

    def test_firm_nonexpansiveness_and_feasible_fixed_points(self):
        m = OmniModel(fixture())
        W = np.diag([.3, 2., .8])
        q = QuadraticProjector(m, W)
        rng = np.random.default_rng(27)
        u, v = rng.normal(size=(2, 100, 3))*m.scale
        projected_diff = q.project(u)-q.project(v)
        lhs = np.einsum("ij,jk,ik->i", projected_diff, W, projected_diff)
        rhs = np.einsum("ij,jk,ik->i", projected_diff, W, u-v)
        self.assertTrue(np.all(lhs <= rhs+1e-8))
        interior = m.radial_clip(u)*.9
        assert_allclose(q.project(interior), interior, atol=0, rtol=0)

    def test_tinympc_state_input_affine_and_margin_contract(self):
        m = OmniModel(fixture())
        rng = np.random.default_rng(32)
        mapping, offset = rng.normal(size=(3, 5)), np.array([.2, -.1, .3])
        z = rng.normal(size=(12, 5))
        for group in ("state", "input"):
            candidate = tinympc_candidate(m, group, mapping, offset, margin=.05)
            assert_allclose(candidate.matrix, m.A@mapping)
            assert_allclose(candidate.bound, .95*m.b-m.A@offset)
            expected = np.maximum(0., (((z@mapping.T+offset)@m.A.T-.95*m.b)/m.b).max(axis=1))
            assert_allclose(candidate.relative_violation(z), expected)
            args = candidate.tinympc_arguments()
            self.assertIsNone(args[2] if group == "state" else args[0])
        with self.assertRaises(ValueError): tinympc_candidate(m, variable="mixed_state_input")
        with self.assertRaises(ValueError): tinympc_candidate(m, margin=1)

    def test_local_parameter_derivatives_and_finite_uncertainty_bound(self):
        m = OmniModel.load(ROOT/"configs/synthetic_asymmetric4.json")
        analytic = row_derivatives(m)
        step = 1e-6
        for field in analytic:
            changed = deepcopy(m.config)
            w = changed["wheels"][0]
            if field == "radius_m": w[field] += step
            elif field == "position_x_m": w["position_xy_m"][0] += step
            elif field == "position_y_m": w["position_xy_m"][1] += step
            else:
                d = np.array(w["rolling_direction_xy"])
                w["rolling_direction_xy"] = (np.array([[np.cos(step), -np.sin(step)],
                                                        [np.sin(step), np.cos(step)]])@d).tolist()
            finite_difference = (OmniModel(changed).J[0]-m.J[0])/step
            assert_allclose(finite_difference, analytic[field][0], rtol=3e-5, atol=1e-3)
        budget = dict(radius_relative=.1, position_component_m=.01, angle_rad=.08, speed_relative=.1)
        rng = np.random.default_rng(28)
        u = rng.normal(size=(100, 3))*m.scale
        bound = utilization_upper_bound(m, u, budget)
        robust = robust_ray_research(m, u, budget)
        for _ in range(12):
            actual = OmniModel(perturb_config(m.config, budget, rng))
            util = np.maximum(actual.inverse(u)/actual.upper, actual.inverse(u)/actual.lower).max(axis=1)
            self.assertTrue(np.all(util <= bound+1e-10))
            self.assertTrue(actual.contains(robust, tolerance=1e-9).all())

    def test_detection_and_ill_conditioned_geometry_diagnostics(self):
        m = OmniModel(fixture())
        self.assertTrue(validation(m)["all_rejected"])
        config = fixture()
        for w in config["wheels"]:
            w["position_xy_m"] = (np.array(w["position_xy_m"])*1e-6).tolist()
        near = OmniModel(config)
        self.assertGreater(near.scaled_condition, 9e5)
        self.assertIn("caution_ill_conditioned", near.numerics)
        self.assertAlmostEqual(near.volume/m.volume, 1e6, delta=.01)
        self.assertTrue(near.contains(near.vertices, tolerance=1e-8).all())
        for w in config["wheels"]: w["position_xy_m"] = [0, 0]
        with self.assertRaises(NumericalGeometryError) as context:
            OmniModel(config)
        self.assertIn("scaled_singular_values", context.exception.diagnostics)

    def test_mppi_batch_shapes_zero_and_bad_metrics(self):
        m = OmniModel(fixture())
        commands = np.random.default_rng(13).normal(size=(2, 3, 4, 3))*m.scale
        for method in ("conservative_box", "maximum_box", "ray", "euclidean", "weighted"):
            candidate = CommandProjector(m, method, np.diag([1., 1., 4.]))
            projected = candidate.project(commands)
            self.assertEqual(projected.shape, commands.shape)
            self.assertTrue(m.contains(projected, tolerance=1e-8).all())
            assert_allclose(candidate.project(np.zeros(3)), 0., atol=1e-12)
        for W in (np.zeros((3, 3)), np.diag([1., -1., 1.]),
                  [[1., 1., 0.], [0., 1., 0.], [0., 0., 1.]]):
            with self.assertRaises(ValueError): QuadraticProjector(m, W)
        with self.assertRaises(ValueError): CommandProjector(m, "weighted")
        with self.assertRaisesRegex(ValueError, "wheel-feasible"):
            CommandProjector(m, "maximum_box", box=m.axis_box)

    def test_stage2_cli_measurement_path_and_invalid_protocol(self):
        protocol = json.loads((ROOT/"configs/stage2_protocol.json").read_text())
        protocol["comparison"].update(seeds=[11], samples_per_seed=40, boundary_directions=4)
        protocol["runtime"].update(single_calls=4, repeats=1, batch_size=4, batch_repeats=1)
        protocol["sensitivity"].update(draws_per_family_level=1, directions=8)
        protocol["regression"].update(seeds=[101], layouts_per_seed=2, commands_per_layout=4, oracle_commands=2)
        with tempfile.TemporaryDirectory() as directory:
            path, output = Path(directory)/"protocol.json", Path(directory)/"output"
            path.write_text(json.dumps(protocol))
            command = [sys.executable, str(ROOT/"stage2.py"), "--protocol", str(path),
                       "--config", str(ROOT/"configs/synthetic_symmetric4.json"), "--output", str(output)]
            run = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout+run.stderr)
            result = json.loads((output/"evidence.json").read_text())
            self.assertTrue(result["passed"])
            self.assertFalse(result["hardware_validated"])
            self.assertFalse(result["comparison"][0]["weighted_equivalent_to_euclidean"])
            self.assertEqual(result["regression"]["layouts"], 2)
            protocol["regression"]["layouts_per_seed"] = 0
            path.write_text(json.dumps(protocol))
            run = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(run.returncode, 2)
            self.assertIn("positive integer", run.stderr)


if __name__ == "__main__":
    unittest.main()
