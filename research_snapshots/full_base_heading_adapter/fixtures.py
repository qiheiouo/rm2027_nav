"""Declared synthetic fixtures. No A/B calibration or recorded hardware data."""
from dataclasses import replace

import numpy as np
from scipy.spatial.transform import Rotation

from adapter import (Calibration, HeadingSample, JointSample, PoseSample, Startup,
                     extrinsic, inv)


def pose(p=(0., 0., 0.), rpy=(0., 0., 0.)):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = p
    return T


def heading_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


def make_case(name="flat", start_rpy=(0., 0., 2.9), rpy=(0., 0., 3.7),
              position=(1.2, -.7, 0.), delta=-1.1, axis_rpy=(0., 0., 0.)):
    # The home/center come from existing canonical unit-test geometry.
    # Decompose it into F,K to explicitly expose the complete chain.
    home = pose((.18, -.11, .35), (.25, -.2, .6))
    F = pose((.03, -.02, .08), axis_rpy)
    K = inv(F) @ home
    cal = Calibration(F, K, .23, "synthetic-v1", "base_fixture", "joint_fixture",
                      "imu_fixture")
    D = pose(rpy=start_rpy)
    truth = inv(D) @ pose(rpy=rpy)
    truth[:3, 3] = position  # translation in complete B0 basis
    raw0 = pose((-.3, .4, .2), (.17, -.23, .51))
    raw1 = raw0 @ inv(home) @ truth @ extrinsic(cal, cal.home_angle_rad + delta)
    s0 = PoseSample(raw0, 1_000_000_000, "host_fixture", "raw-stable-1",
                    "raw_fixture", "imu_fixture")
    s1 = replace(s0, T_raw_sensor=raw1, stamp_ns=2_000_000_000)
    h0 = HeadingSample(heading_of(D[:3, :3]), s0.stamp_ns, s0.clock_id,
                       "boot-1/reset-0", "heading_fixture")
    h1 = replace(h0, yaw_rad=heading_of((D @ truth)[:3, :3]), stamp_ns=s1.stamp_ns)
    startup = Startup(s0, h0, D[:3, :3], D[:3, :3].T @ [0., 0., -1.],
                      s0.stamp_ns, s0.stamp_ns, s0.clock_id, True,
                      cal.home_angle_rad, cal.calibration_id, "odom_research_fixture")
    joint = JointSample(cal.home_angle_rad + delta, s1.stamp_ns, s1.clock_id,
                        cal.joint_frame, cal.calibration_id)
    return dict(name=name, raw=s1, heading=h1, calibration=cal, startup=startup,
                joint=joint, truth_base0=truth, truth_odom=D @ truth, delta=delta,
                home=home, center=F[:3, 3])


def cases():
    return [
        make_case("flat_gimbal_heading_wrap"),
        make_case("flat_pi_boundary", start_rpy=(0, 0, 3.10), rpy=(0, 0, -3.10),
                  position=(.1, .2, 0), delta=2.8),
        make_case("tilt_from_level_start", start_rpy=(0, 0, .3), rpy=(.3, -.25, 1.1),
                  position=(1.2, -.7, .2)),
        make_case("tilted_start_home", start_rpy=(.2, -.18, 2.9), rpy=(.2, -.18, 2.9),
                  position=(0, 0, 0), delta=0),
        make_case("tilted_start_motion", start_rpy=(.2, -.18, 2.9), rpy=(.32, -.12, -2.6),
                  position=(.7, -.2, .15), delta=.9),
        make_case("sideways_continuum", start_rpy=(0, 0, 0), rpy=(np.pi/2, 0, .4),
                  position=(.7, -.2, .15), delta=.6),
        make_case("near_singular", start_rpy=(0, 0, 0), rpy=(np.deg2rad(89), 0, .4),
                  position=(.7, -.2, .15), delta=.6),
        make_case("tilted_axis_two_roots", start_rpy=(0, 0, 0), rpy=(0, .1, 0),
                  axis_rpy=(0, np.pi/3, 0), position=(.7, -.2, .15), delta=.6),
    ]


def pose_error(estimate, truth):
    return dict(position_error_m=float(np.linalg.norm(estimate[:3, 3] - truth[:3, 3])),
                rotation_error_deg=float(np.rad2deg(np.linalg.norm(
                    Rotation.from_matrix(truth[:3, :3].T @ estimate[:3, :3]).as_rotvec()))))
