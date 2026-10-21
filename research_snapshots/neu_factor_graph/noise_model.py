#!/usr/bin/env python3
"""Original first-order covariance algebra for the private right-SE(3) graph.

All charts/conditioning must be declared by the caller. These operations do
not estimate hardware noise, convert a legacy ROS covariance blindly, or
infer missing cross-time/cross-source correlation. Order: [rho, phi], m/rad.
"""
import numpy as np

from backend import inv, rigid, skew


def covariance(value, dimension=6):
    C = np.asarray(value, dtype=float)
    if C.shape != (dimension, dimension) or not np.isfinite(C).all():
        raise ValueError("invalid covariance shape or values")
    scale = max(float(np.linalg.norm(C, ord=2)), np.finfo(float).tiny)
    if np.max(abs(C-C.T)) > 1e-10*scale:
        raise ValueError("covariance is not symmetric")
    C = (C+C.T)/2
    if np.linalg.eigvalsh(C)[0] < -1e-10*scale:
        raise ValueError("covariance is not positive semidefinite")
    return C


def adjoint(T):
    T = rigid(T)
    R = T[:3, :3]
    A = np.zeros((6, 6))
    A[:3, :3] = R
    A[:3, 3:] = skew(T[:3, 3])@R
    A[3:, 3:] = R
    return A


def native_mixed_to_right(T_world_sensor, P_pos_rotation):
    """P chart = [additive dp_world, right dphi_sensor], actual IKFoM order.

    Parent-basis transforms are conditioned deterministic. No claim to have
    propagated uncertainty of the estimated startup origin or calibration.
    This requires native current-state P, not the legacy published 36 fields.
    """
    T = rigid(T_world_sensor)
    J = np.eye(6)
    J[:3, :3] = T[:3, :3].T
    return J@covariance(P_pos_rotation)@J.T


def sensor_to_base_right(T_base_sensor, sensor_covariance, *, extrinsic_conditioned):
    """For T_odom_base=T_odom_sensor*inverse(T_base_sensor).

    A measured, timestamp-matched extrinsic is treated as deterministic only
    with the explicit conditioning flag. Unknown gimbal/calibration noise is
    not silently made zero; this minimum function cannot model it yet.
    """
    if not extrinsic_conditioned:
        raise ValueError("extrinsic / gimbal uncertainty required; not implemented as zero")
    A = adjoint(T_base_sensor)
    return A@covariance(sensor_covariance)@A.T


def relative_right_covariance(T_i_j, P_i, P_j, cross_i_j=None, *, independent=False):
    """C_delta for inverse(T_i)*T_j, including known joint covariance.

    d_delta = -Ad(inverse(T_i_j))*d_i + d_j. Cross_i_j is E[d_i*d_j.T]
    in the two respective right-body charts. Missing correlation is an error;
    callers may explicitly declare an independent approximation instead.
    """
    Pi, Pj = covariance(P_i), covariance(P_j)
    if cross_i_j is None:
        if not independent:
            raise ValueError("cross-time covariance unknown; cannot infer increment noise from marginals")
        cross = np.zeros((6, 6))
    else:
        if independent:
            raise ValueError("choose known correlation or declared independence")
        cross = np.asarray(cross_i_j, dtype=float)
        if cross.shape != (6, 6) or not np.isfinite(cross).all():
            raise ValueError("invalid cross covariance")
    covariance(np.block([[Pi, cross], [cross.T, Pj]]), dimension=12)
    A = adjoint(inv(rigid(T_i_j)))
    return covariance(A@Pi@A.T+Pj-A@cross-cross.T@A.T)


def sqrt_information(C):
    """PD covariance -> L with L.T*L=C^-1. No variance floor or pseudo inverse.

    Zero covariance is not interpreted as an unobserved direction or an exact
    constraint. A partially observed state requires an explicit information
    model; this function refuses it rather than inventing confidence.
    """
    C = covariance(C)
    try:
        lower = np.linalg.cholesky(C)
    except np.linalg.LinAlgError as exc:
        raise ValueError("singular covariance cannot supply a full-state noise model") from exc
    return np.linalg.solve(lower, np.eye(6))
