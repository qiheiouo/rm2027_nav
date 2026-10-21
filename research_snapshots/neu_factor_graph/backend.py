#!/usr/bin/env python3
"""Original batch SE(3) odom/map graph; private JSON only, no ROS authority.

T_A_B maps B coordinates into A. Tangent order is [rho_xyz, phi_xyz], m/rad.
Right exponential retraction. L is sqrt information: Lambda = L.T @ L.
No truth labels, covariance extraction, candidate selection, recovery or reset.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation


def skew(v):
    v = np.asarray(v)
    out = np.zeros(v.shape[:-1] + (3, 3))
    out[..., 0, 1], out[..., 0, 2] = -v[..., 2], v[..., 1]
    out[..., 1, 0], out[..., 1, 2] = v[..., 2], -v[..., 0]
    out[..., 2, 0], out[..., 2, 1] = -v[..., 1], v[..., 0]
    return out


def exp6(x):
    x = np.asarray(x, dtype=float)
    shape = x.shape[:-1]
    phi = x[..., 3:]
    t2 = np.sum(phi * phi, axis=-1)
    t = np.sqrt(t2)
    a = np.full(shape, .5)
    b = np.full(shape, 1 / 6)
    small = t2 < 1e-8
    a[small] = .5 - t2[small] / 24 + t2[small] ** 2 / 720
    b[small] = 1 / 6 - t2[small] / 120 + t2[small] ** 2 / 5040
    np.divide(1 - np.cos(t), t2, out=a, where=~small)
    np.divide(t - np.sin(t), t2 * t, out=b, where=~small)
    w = skew(phi)
    v = np.eye(3) + a[..., None, None] * w + b[..., None, None] * (w @ w)
    out = np.broadcast_to(np.eye(4), shape + (4, 4)).copy()
    out[..., :3, :3] = Rotation.from_rotvec(phi.reshape(-1, 3)).as_matrix().reshape(shape + (3, 3))
    out[..., :3, 3] = (v @ x[..., :3, None])[..., 0]
    return out


def inv(T):
    T = np.asarray(T)
    out = np.broadcast_to(np.eye(4), T.shape).copy()
    r = T[..., :3, :3].swapaxes(-1, -2)
    out[..., :3, :3] = r
    out[..., :3, 3] = -(r @ T[..., :3, 3, None])[..., 0]
    return out


def log6(T):
    T = np.asarray(T)
    shape = T.shape[:-2]
    if T.size == 0:
        return np.empty(shape + (6,))
    phi = Rotation.from_matrix(T[..., :3, :3].reshape(-1, 3, 3)).as_rotvec().reshape(shape + (3,))
    t2 = np.sum(phi * phi, axis=-1)
    t = np.sqrt(t2)
    d = np.full(shape, 1 / 12)
    small = t2 < 1e-8
    d[small] = 1 / 12 + t2[small] / 720 + t2[small] ** 2 / 30240
    # Stable through pi; principal logarithm is still a local chart, not a global solver.
    np.divide(1 - .5 * t / np.tan(np.maximum(t, 1e-30) / 2), t2, out=d, where=~small)
    w = skew(phi)
    vinv = np.eye(3) - .5 * w + d[..., None, None] * (w @ w)
    rho = (vinv @ T[..., :3, 3, None])[..., 0]
    return np.concatenate([rho, phi], axis=-1)


def rigid(value):
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError("invalid transform")
    r = T[:3, :3]
    if (not np.allclose(T[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(r) - 1) > 1e-6):
        raise ValueError("transform is not SE(3)")
    return T


def information(value, require_full=False):
    L = np.asarray(value, dtype=float)
    if L.shape != (6, 6) or not np.isfinite(L).all() or np.linalg.norm(L) == 0:
        raise ValueError("invalid sqrt information")
    if require_full and np.linalg.matrix_rank(L) != 6:
        raise ValueError("minimum prototype requires full-rank odom noise model")
    return L


def validate(data):
    context = data["context"]
    if any(data.get("reserved_factors", {}).values()):
        raise ValueError("UWB / loop / retrieval are reserved, not implemented factors")
    for name in ("session_id", "odom_epoch", "numerical_basis_revision", "map_revision", "calibration_revision"):
        if not context.get(name):
            raise ValueError("missing context: " + name)
    if context.get("frames") != {"map": "map", "odom": "odom", "base": "base_link"}:
        raise ValueError("requires canonical base poses, not sensor poses")
    if context.get("input_kind") not in ("synthetic", "verified_recording"):
        raise ValueError("unknown numerical / calibration input basis")
    samples = data["samples"]
    if not samples:
        raise ValueError("no odometry")
    ids, stamps = [s["id"] for s in samples], [s["timestamp_ns"] for s in samples]
    if (len(set(ids)) != len(ids) or any(not isinstance(t, int) or t <= 0 for t in stamps)
            or any(b <= a for a, b in zip(stamps, stamps[1:]))):
        raise ValueError("duplicate ID or nonmonotonic time; split epochs before this prototype")
    O = np.array([rigid(s["T_odom_base"]) for s in samples])
    if "odom_factors" in data:
        if "odom_sqrt_information" in data:
            raise ValueError("choose explicit per-edge or common odom information")
        edges = data["odom_factors"]
        if len(edges) != len(samples)-1:
            raise ValueError("one information record required per consecutive odom pair")
        edge_ids, weights = set(), []
        for i, edge in enumerate(edges):
            if (edge["from_id"] != ids[i] or edge["to_id"] != ids[i+1]
                    or edge["from_timestamp_ns"] != stamps[i]
                    or edge["to_timestamp_ns"] != stamps[i+1]
                    or edge["context"] != context):
                raise ValueError("odom factor node/time/basis context mismatch")
            if edge["id"] in edge_ids or not edge.get("noise_model_origin"):
                raise ValueError("duplicate odom edge or missing noise provenance")
            edge_ids.add(edge["id"])
            weights.append(information(edge["sqrt_information"], require_full=True))
        Lod = np.array(weights).reshape(-1, 6, 6)
    else:
        common = information(data["odom_sqrt_information"], require_full=True)
        if not data.get("odom_noise_model_origin"):
            raise ValueError("missing incremental odom noise-model provenance")
        Lod = np.broadcast_to(common, (len(samples)-1, 6, 6)).copy()
    factor_ids, query_ids, factors = set(), set(), []
    for f in data.get("scan_factors", []):
        if f["context"] != context:
            raise ValueError("map/basis/epoch context mismatch")
        if f["node_id"] not in ids:
            raise ValueError("unknown factor node")
        idx = ids.index(f["node_id"])
        if f["timestamp_ns"] != stamps[idx]:
            raise ValueError("scan/odom timestamps must be pre-associated exactly")
        if f["id"] in factor_ids or f["query_id"] in query_ids:
            raise ValueError("duplicate factor or same query reused as independent information")
        if f["basin_status"] != "single_basin_assumed":
            raise ValueError("caller must supply one explicit basin hypothesis; no best-candidate selection")
        if not f.get("noise_model_origin"):
            raise ValueError("missing noise-model provenance; fitness is not covariance")
        factor_ids.add(f["id"])
        query_ids.add(f["query_id"])
        factors.append((idx, rigid(f["T_map_base"]), information(f["sqrt_information"])))
    if factors and not any(np.linalg.matrix_rank(L) == 6 for _, _, L in factors):
        raise ValueError("minimum global graph requires an explicit full-pose anchor")
    return O, Lod, factors


def solve(data, initial=None):
    start = time.perf_counter()
    O, Lod, factors = validate(data)
    n = len(O)
    global_anchor = bool(factors)
    if initial is None:
        if factors:
            anchor = next((i, S) for i, S, L in factors if np.linalg.matrix_rank(L) == 6)
            C = anchor[1] @ inv(O[anchor[0]])
            basis = C @ O
        else:
            basis = O.copy()
    else:
        basis = np.array([rigid(T) for T in initial])
        if basis.shape != O.shape:
            raise ValueError("initial trajectory size mismatch")
    if not global_anchor:
        basis[0] = O[0]
    # Relative-only graph pins X0 to O0 solely to remove gauge. No global origin claim.
    offset = 0 if global_anchor else 1
    zinv = inv(inv(O[:-1]) @ O[1:])
    map_nodes = np.array([i for i, _, _ in factors], dtype=int)
    sinv = np.array([inv(S) for _, S, _ in factors])
    Lmap = np.array([L for _, _, L in factors])

    def poses(x):
        delta = np.zeros((n, 6))
        delta[offset:] = x.reshape(-1, 6)
        return basis @ exp6(delta)

    def residual(x):
        X = poses(x)
        ro = np.einsum("nij,nj->ni", Lod, log6(zinv @ inv(X[:-1]) @ X[1:]))
        if not factors:
            return ro.ravel()
        rm = np.einsum("nij,nj->ni", Lmap, log6(sinv @ X[map_nodes]))
        return np.concatenate([ro.ravel(), rm.ravel()])

    pattern = lil_matrix((6 * (n - 1 + len(factors)), 6 * (n - offset)), dtype=int)
    for i in range(n - 1):
        for node in (i, i + 1):
            if node >= offset:
                pattern[6*i:6*(i+1), 6*(node-offset):6*(node-offset+1)] = 1
    for j, (node, _, _) in enumerate(factors):
        pattern[6*(n-1+j):6*(n+j), 6*(node-offset):6*(node-offset+1)] = 1
    if n == offset:
        X, cost, success, nfev, optimality = basis, 0., True, 0, 0.
    else:
        solution = least_squares(residual, np.zeros(6*(n-offset)), method="trf",
                                 jac_sparsity=pattern.tocsr(), loss="linear", max_nfev=100,
                                 ftol=1e-8, xtol=1e-8, gtol=1e-8,
                                 tr_options={"atol": 1e-10, "btol": 1e-10})
        X, cost, success = poses(solution.x), solution.cost, bool(solution.success)
        nfev, optimality = solution.nfev, float(solution.optimality)
    C = X @ inv(O)
    return {"schema_version": 1, "context": data["context"], "active_accept": False,
            "status": "research_solution" if global_anchor else "relative_only_no_global_anchor",
            "global_uniqueness": "NOT_ESTABLISHED", "posterior_covariance": None,
            "cross_factor_covariance": None,
            "T_map_base": X.tolist() if global_anchor else None,
            "T_relative_base": X.tolist() if not global_anchor else None,
            "T_map_odom": C.tolist() if global_anchor else None,
            "source_timestamps_ns": [s["timestamp_ns"] for s in data["samples"]],
            "node_ids": [s["id"] for s in data["samples"]],
            "scan_factor_ids": [f["id"] for f in data.get("scan_factors", [])],
            "odom_noise_model": "per_edge" if "odom_factors" in data else "common_nominal",
            "odom_noise_origins": ([f["noise_model_origin"] for f in data["odom_factors"]]
                                   if "odom_factors" in data else [data["odom_noise_model_origin"]]),
            "solver_success": success, "cost": float(cost), "nfev": int(nfev),
            "optimality": optimality, "solve_ms": (time.perf_counter()-start)*1000}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = solve(json.loads(args.input.read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
