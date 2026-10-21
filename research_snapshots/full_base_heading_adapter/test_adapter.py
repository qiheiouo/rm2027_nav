"""Geometry/timing failure controls; all inputs are synthetic and declared."""
from dataclasses import replace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from adapter import (Policy, Status, extrinsic, inv, rz, solve,
                     startup_attitude_from_gravity, wrap)
from fixtures import cases, make_case, pose, pose_error
from native import planar


def recover(c, **kwargs):
    return solve(c["raw"], c["heading"], c["calibration"], c["startup"], **kwargs)


class AdapterTests(unittest.TestCase):
    def assert_pose(self, result, truth, tol=1e-9):
        self.assertEqual(result.status, Status.SOLVED, result.reason)
        np.testing.assert_allclose(result.T_odom_base, truth, atol=tol, rtol=0)
        self.assertFalse(result.active_accept)
        self.assertIsNone(result.covariance)

    def assert_rejected(self, result, status):
        self.assertEqual(result.status, status, result.reason)
        self.assertIsNone(result.T_odom_base)
        self.assertIsNone(result.T_base0_base)
        self.assertIsNone(result.joint_angle_rad)

    def test_flat_parity_with_actual_cpp(self):
        samples = [make_case(start_rpy=(0, 0, .3), rpy=(0, 0, h), delta=q)
                   for h, q in [(-3.13, .6), (.4, -1.1), (2.8, 3.14), (3.7, -4.2)]]
        for c, native in zip(samples, planar(samples)):
            result = recover(c)
            self.assert_pose(result, c["truth_odom"])
            np.testing.assert_allclose(result.T_base0_base, np.array(native["pose"]).reshape(4, 4),
                                       atol=1e-9, rtol=0)
            self.assertAlmostEqual(wrap(result.joint_angle_rad - c["calibration"].home_angle_rad),
                                   native["joint_delta_rad"])

    def test_yaw_wrap_and_heading_origin(self):
        c = cases()[1]
        result = recover(c)
        self.assert_pose(result, c["truth_odom"])
        shifted = dict(c)
        alpha = .73
        R0 = rz(alpha) @ c["startup"].R_reference_base0
        shifted["startup"] = replace(c["startup"], R_reference_base0=R0,
                                      heading=replace(c["startup"].heading,
                                                      yaw_rad=wrap(c["startup"].heading.yaw_rad + alpha)))
        shifted["heading"] = replace(c["heading"], yaw_rad=wrap(c["heading"].yaw_rad + alpha + 4*np.pi))
        r2 = recover(shifted)
        D = pose(rpy=(0, 0, alpha))
        self.assert_pose(r2, D @ result.T_odom_base)
        np.testing.assert_allclose(r2.T_base0_base, result.T_base0_base, atol=1e-9)

    def test_pure_gimbal_rotation_does_not_move_chassis(self):
        c = make_case(rpy=(0, 0, 2.9), position=(0, 0, 0), delta=1.4)
        self.assert_pose(recover(c), c["truth_odom"])
        self.assertAlmostEqual(wrap(recover(c).joint_angle_rad - c["calibration"].home_angle_rad), 1.4)

    def test_chassis_roll_pitch_and_lever_arm(self):
        c = cases()[2]
        result = recover(c)
        self.assert_pose(result, c["truth_odom"])
        baseline = np.array(planar([c])[0]["pose"]).reshape(4, 4)
        error = pose_error(baseline, c["truth_base0"])
        self.assertAlmostEqual(error["position_error_m"], .14601319562190648)
        self.assertAlmostEqual(error["rotation_error_deg"], 22.340215785387493)

    def test_tilted_startup_keeps_gravity_aligned_odom(self):
        c = cases()[4]
        self.assert_pose(recover(c), c["truth_odom"])
        r = solve(c["startup"].raw, c["startup"].heading, c["calibration"], c["startup"])
        np.testing.assert_allclose(r.T_base0_base, np.eye(4), atol=1e-9)
        np.testing.assert_allclose(r.T_odom_base[:3, :3], c["startup"].R_reference_base0, atol=1e-9)

    def test_full_startup_attitude_can_be_explicitly_constructed_from_gravity(self):
        c = cases()[4]
        reconstructed = startup_attitude_from_gravity(
            c["startup"].gravity_down_base0, c["startup"].heading.yaw_rad)
        np.testing.assert_allclose(reconstructed, c["startup"].R_reference_base0, atol=1e-9)
        with self.assertRaises(ValueError):
            startup_attitude_from_gravity([1., 0., 0.], 0.)
        with self.assertRaises(ValueError):
            startup_attitude_from_gravity([0., 0., 0.], 0.)

    def test_known_joint_resolves_sideways_continuum(self):
        c = cases()[5]
        self.assert_pose(recover(c, joint=c["joint"]), c["truth_odom"])

    def test_ambiguous_decomposition_has_distinct_exact_witnesses(self):
        c = cases()[5]
        self.assert_rejected(recover(c), Status.AMBIGUOUS)
        r1 = recover(c, joint=c["joint"])
        other = replace(c["joint"], angle_rad=c["calibration"].home_angle_rad + 1.0)
        r2 = recover(c, joint=other)
        self.assertEqual(r2.status, Status.SOLVED)
        self.assertGreater(pose_error(r2.T_odom_base, r1.T_odom_base)["rotation_error_deg"], 20)
        np.testing.assert_allclose(r1.T_odom_base @ extrinsic(c["calibration"], c["joint"].angle_rad),
                                   r2.T_odom_base @ extrinsic(c["calibration"], other.angle_rad), atol=1e-9)

    def test_tilted_axis_two_directed_roots_remain_ambiguous(self):
        c = cases()[7]
        result = recover(c)
        self.assert_rejected(result, Status.AMBIGUOUS)
        self.assertEqual(len(result.diagnostics["candidates"]), 2)
        self.assert_pose(recover(c, joint=c["joint"]), c["truth_odom"])

    def test_known_tilted_axis_and_complete_mount(self):
        c = make_case(start_rpy=(.2, -.18, 2.9), rpy=(.32, -.12, -2.6),
                      axis_rpy=(.15, -.2, .7), position=(.7, -.2, .15), delta=.9)
        self.assert_pose(recover(c, joint=c["joint"]), c["truth_odom"])

    def test_startup_joint_must_agree_with_confirmed_home(self):
        c = cases()[7]
        s = c["startup"]
        contradictory_joint = replace(c["joint"], angle_rad=c["calibration"].home_angle_rad + np.pi,
                                      stamp_ns=s.raw.stamp_ns)
        r = solve(s.raw, s.heading, c["calibration"], s, joint=contradictory_joint)
        self.assert_rejected(r, Status.INVALID_CALIBRATION)

    def test_heading_incompatible_with_tilted_axis(self):
        c = make_case(start_rpy=(0, 0, 0), rpy=(0, 0, 0), delta=0,
                      axis_rpy=(0, np.pi/3, 0))
        c["heading"] = replace(c["heading"], yaw_rad=np.pi/2)
        self.assert_rejected(recover(c), Status.INVALID_INPUT)

    def test_tangent_root_is_ill_conditioned(self):
        c = make_case(start_rpy=(0, 0, 0), rpy=(0, 0, 0), delta=0,
                      axis_rpy=(0, np.pi/3, 0))
        c["heading"] = replace(c["heading"], yaw_rad=np.arctan(1/np.sqrt(2)))
        self.assert_rejected(recover(c), Status.ILL_CONDITIONED)

    def test_seeded_full_attitude_property_controls(self):
        rng = np.random.default_rng(20261007)
        for _ in range(100):
            rpy0 = rng.uniform([-.4, -.4, -np.pi], [.4, .4, np.pi])
            rpy1 = rng.uniform([-1.2, -.8, -np.pi], [1.2, .8, np.pi])
            c = make_case(start_rpy=rpy0, rpy=rpy1, position=rng.uniform(-2, 2, 3),
                          delta=rng.uniform(-4*np.pi, 4*np.pi))
            self.assert_pose(recover(c), c["truth_odom"])

    def test_near_singular_sensitivity_is_flagged_and_measured(self):
        c = cases()[6]
        self.assert_rejected(recover(c), Status.ILL_CONDITIONED)
        loose = Policy(min_constraint_derivative=1e-6, max_heading_gain=1e6)
        nominal = recover(c, policy=loose)
        perturbed = dict(c, heading=replace(c["heading"], yaw_rad=c["heading"].yaw_rad + 1e-7))
        response = recover(perturbed, policy=loose)
        rotation_response = np.linalg.norm(Rotation.from_matrix(
            nominal.T_odom_base[:3, :3].T @ response.T_odom_base[:3, :3]).as_rotvec()) / 1e-7
        self.assertAlmostEqual(rotation_response, 1/np.cos(np.deg2rad(89)), places=4)
        self.assertGreater(rotation_response, 57)

    def test_timestamp_mismatch_rejected(self):
        c = make_case()
        c["heading"] = replace(c["heading"], stamp_ns=c["raw"].stamp_ns - 20_000_000)
        self.assert_rejected(recover(c), Status.INVALID_TIME)

    def test_stale_heading_cannot_be_safely_fixed_by_relabeling_time(self):
        c = cases()[2]
        c["heading"] = replace(c["heading"], yaw_rad=c["heading"].yaw_rad - .02*6.,
                               stamp_ns=c["raw"].stamp_ns - 20_000_000)
        self.assert_rejected(recover(c), Status.INVALID_TIME)
        c["heading"] = replace(c["heading"], stamp_ns=c["raw"].stamp_ns)
        result = recover(c)
        self.assertEqual(result.status, Status.SOLVED)
        self.assertGreater(pose_error(result.T_odom_base, c["truth_odom"])["rotation_error_deg"], 5)

    def test_time_gate_boundary_does_not_retimestamp(self):
        c = make_case()
        c["heading"] = replace(c["heading"], stamp_ns=c["raw"].stamp_ns - 1_000_000)
        r = recover(c)
        self.assertEqual(r.status, Status.SOLVED)
        self.assertEqual(r.diagnostics["heading_sensor_skew_ns"], -1_000_000)
        c["heading"] = replace(c["heading"], stamp_ns=c["heading"].stamp_ns - 1)
        self.assert_rejected(recover(c), Status.INVALID_TIME)

    def test_joint_timestamp_mismatch_rejected(self):
        c = make_case()
        self.assert_rejected(recover(c, joint=replace(c["joint"], stamp_ns=1_900_000_000)),
                             Status.INVALID_TIME)

    def test_startup_time_mismatch_rejected(self):
        c = make_case()
        c["startup"] = replace(c["startup"], attitude_stamp_ns=900_000_000)
        self.assert_rejected(recover(c), Status.INVALID_TIME)

    def test_clock_and_epoch_mismatch_rejected(self):
        for change in [dict(clock_id="other-clock"), dict(raw_epoch="after-zero-start")]:
            c = make_case()
            c["raw"] = replace(c["raw"], **change)
            self.assert_rejected(recover(c), Status.INVALID_TIME)
        c = make_case()
        c["heading"] = replace(c["heading"], heading_epoch="boot-1/reset-1")
        self.assert_rejected(recover(c), Status.INVALID_TIME)

    def test_invalid_source_time(self):
        for stamp in [0, -1, 1.5, True, 500_000_000, 1_000_000_000]:
            c = make_case()
            c["raw"] = replace(c["raw"], stamp_ns=stamp)
            self.assert_rejected(recover(c), Status.INVALID_TIME)

    def test_missing_or_false_home_rejected(self):
        for change in [dict(home_confirmed=False), dict(home_joint_angle_rad=1.0),
                       dict(calibration_id="different-calibration")]:
            c = make_case()
            c["startup"] = replace(c["startup"], **change)
            self.assert_rejected(recover(c), Status.INVALID_CALIBRATION)

    def test_missing_startup_full_attitude_rejected(self):
        c = make_case()
        c["startup"] = replace(c["startup"], R_reference_base0=None)
        self.assert_rejected(recover(c), Status.INVALID_CALIBRATION)

    def test_startup_gravity_and_heading_convention_checked(self):
        for change in [dict(gravity_down_base0=np.array([0., 0., 1.])),
                       dict(R_reference_base0=rz(.5))]:
            c = make_case()
            c["startup"] = replace(c["startup"], **change)
            self.assert_rejected(recover(c), Status.INVALID_CALIBRATION)

    def test_consistent_but_wrong_startup_reference_cannot_be_detected(self):
        c = cases()[4]
        wrong = rz(c["startup"].heading.yaw_rad)
        c["startup"] = replace(c["startup"], R_reference_base0=wrong,
                               gravity_down_base0=wrong.T @ [0., 0., -1.])
        r = recover(c)
        self.assertEqual(r.status, Status.SOLVED)
        self.assertGreater(pose_error(r.T_odom_base, c["truth_odom"])["rotation_error_deg"], 10.)

    def test_invalid_calibration_rotation_and_model_rejected(self):
        for change in [dict(T_base_joint=np.zeros((4, 4))), dict(model="two_axis"),
                       dict(sensor_frame="base_fixture"), dict(home_angle_rad=np.nan)]:
            c = make_case()
            c["calibration"] = replace(c["calibration"], **change)
            self.assert_rejected(recover(c), Status.INVALID_CALIBRATION)

    def test_invalid_raw_pose_input_and_frame_rejected(self):
        for change in [dict(T_raw_sensor=np.full((4, 4), np.nan)), dict(sensor_frame="lidar_origin"),
                       dict(raw_parent="other-origin"), dict(valid=False)]:
            c = make_case()
            c["raw"] = replace(c["raw"], **change)
            self.assert_rejected(recover(c), Status.INVALID_INPUT)

    def test_invalid_heading_or_joint_rejected(self):
        for change in [dict(yaw_rad=np.nan), dict(online=False), dict(reference_frame="other-world")]:
            c = make_case()
            c["heading"] = replace(c["heading"], **change)
            self.assert_rejected(recover(c), Status.INVALID_INPUT)
        c = make_case()
        self.assert_rejected(recover(c, joint=replace(c["joint"], calibration_id="wrong")),
                             Status.INVALID_CALIBRATION)
        self.assert_rejected(recover(c, joint=replace(c["joint"], angle_rad=c["joint"].angle_rad + .2)),
                             Status.INVALID_INPUT)

    def test_raw_parent_numeric_basis_invariance(self):
        c = make_case()
        change = pose((1, 2, 3), (.1, -.2, .4))
        c["raw"] = replace(c["raw"], T_raw_sensor=change @ c["raw"].T_raw_sensor)
        c["startup"] = replace(c["startup"], raw=replace(c["startup"].raw,
                               T_raw_sensor=change @ c["startup"].raw.T_raw_sensor))
        self.assert_pose(recover(c), c["truth_odom"])

    def test_sensor_initial_helper_requires_distinct_startup_extrinsic(self):
        c = make_case(start_rpy=(0, 0, 0), rpy=(0, 0, 0), position=(0, 0, 0), delta=.6)
        actual = np.array(planar([c])[0]["sensor_initial_dynamic_pose"]).reshape(4, 4)
        self.assertGreater(pose_error(actual, c["truth_base0"])["rotation_error_deg"], 30)
        proper = c["home"] @ inv(c["startup"].raw.T_raw_sensor) @ c["raw"].T_raw_sensor @ inv(
            extrinsic(c["calibration"], c["joint"].angle_rad))
        np.testing.assert_allclose(proper, c["truth_base0"], atol=1e-9)
        self.assert_pose(recover(c), c["truth_odom"])


if __name__ == "__main__":
    unittest.main()
