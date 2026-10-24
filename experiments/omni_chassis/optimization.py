"""Research-only maximum boxes and metric projections; no controller/solver IO."""

from dataclasses import dataclass
from itertools import combinations

import numpy as np
from scipy.optimize import minimize, nnls

from model import Box, array, vectors


class OptimizationError(RuntimeError):
    pass


def maximum_box(model, centre="contains_zero"):
    """Globally concave log-volume problem: A c + |A| h <= b, h > 0.

    centre: 'origin', 'free', or 'contains_zero'. Return a primal box and a
    numerical weak-duality upper bound, not just a solver success flag.
    """
    if centre not in ("origin", "free", "contains_zero"):
        raise ValueError("unknown box centre policy")
    length = np.maximum(abs(model.bounding_box.lower), abs(model.bounding_box.upper))
    width_length = ((model.inner_box.upper-model.inner_box.lower)/2 if centre != "origin" else
                    np.minimum(-model.inner_box.lower, model.inner_box.upper))
    width_map = width_length/length
    B = model.A * length / model.b[:, None]
    if centre == "origin":
        C, D = np.empty((len(B), 0)), abs(B)*width_map
        start = np.ones(3)*.95
    else:
        C, D = B.copy(), abs(B)*width_map
        start = np.r_[(model.inner_box.upper + model.inner_box.lower) / (2 * length),
                      np.ones(3)*.95]
    rhs = np.ones(len(B))
    if centre == "contains_zero":
        # c-h<=0, -c-h<=0: every zero component remains inside the box.
        C = np.vstack((C, np.eye(3), -np.eye(3)))
        D = np.vstack((D, -np.diag(width_map), -np.diag(width_map)))
        rhs = np.r_[rhs, np.zeros(6)]
    G = np.column_stack((C, D))
    nc = C.shape[1]

    def objective(y):
        return -np.log(y[nc:]).sum()

    def gradient(y):
        return np.r_[np.zeros(nc), -1 / y[nc:]]

    result = minimize(objective, start, jac=gradient, method="SLSQP",
                      bounds=[(None, None)] * nc + [(1e-12, None)] * 3,
                      constraints=[dict(type="ineq", fun=lambda y: rhs-G@y, jac=lambda y: -G)],
                      options=dict(ftol=1e-12, maxiter=300))
    y = result.x
    raw_violation = float(np.max(G @ y - rhs))
    if not np.all(np.isfinite(y)) or np.any(y[nc:] <= 0) or raw_violation > 1e-8:
        raise OptimizationError(f"maximum box unresolved: {result.message}, violation={raw_violation}")
    c = y[:nc] if nc else np.zeros(3)
    h = y[nc:]*width_map
    # Only remove roundoff-sized primal violations; record the raw violation.
    # A centre outside P is an error, never silently replaced by another centre.
    room = 1 - B @ c
    if np.min(room) <= 0:
        raise OptimizationError("maximum-box centre is not strictly inside P")
    shrink = min(1., float(np.min(room / (abs(B) @ h))))
    h = h * shrink
    if centre == "contains_zero" and np.any(abs(c) > h + 1e-9):
        raise OptimizationError("zero command outside computed box")
    if centre == "contains_zero":
        lo, hi = np.minimum(c-h, 0.), np.maximum(c+h, 0.)
        c, h = (lo+hi)/2, (hi-lo)/2
    # Snap zero faces then remove only floating-point excess by a homothety
    # through zero. Compute the certificate for this final feasible box.
    corner_load = float((Box(c-h, c+h).corners @ B.T).max())
    if corner_load > 1:
        c, h = c/corner_load, h/corner_load
    scaled_width = h/width_map
    y = np.r_[c, scaled_width] if nc else scaled_width

    # Recover nonnegative multipliers on approximately active rows. The explicit
    # centre support term below makes the upper bound valid even if stationarity
    # in c is only approximately satisfied (centres lie in P's bounding box).
    slack = rhs - G @ y
    active = slack < 1e-6
    target = np.r_[np.zeros(nc), 1/scaled_width]
    multiplier, _ = nnls(G[active].T, target, maxiter=1000)
    lam = np.zeros(len(G))
    lam[active] = multiplier
    q = D.T @ lam
    if np.any(q <= 0):
        raise OptimizationError("no positive-width dual certificate for maximum box")
    centre_support = 0.
    if nc:
        residual = C.T @ lam
        centre_support = float(np.maximum(-residual * model.bounding_box.lower / length,
                                           -residual * model.bounding_box.upper / length).sum())
    dual = float(lam @ rhs + centre_support - np.log(q).sum() - 3)
    primal = float(np.log(scaled_width).sum())
    if dual < primal-1e-8:
        raise OptimizationError("weak-duality numerical check failed")
    gap = max(0., dual-primal)
    if gap > 1e-5:
        raise OptimizationError(f"maximum box dual gap too large: {gap}")
    box = Box((c-h)*length, (c+h)*length)
    if not np.all(model.contains(box.corners, tolerance=1e-8)):
        raise OptimizationError("maximum-box corner certification failed")
    return dict(box=box, centre_policy=centre, log_volume_gap_bound=gap,
                volume_upper_bound=box.volume * np.exp(gap),
                solver_success=bool(result.success), solver_status=int(result.status),
                solver_message=str(result.message), certificate_accepted=True,
                raw_primal_violation=raw_violation, iterations=int(result.nit),
                dual_stationarity_residual=float(np.max(abs(G.T @ lam - target))),
                contains_zero=bool(np.all((box.lower <= 1e-9) & (box.upper >= -1e-9))))


class QuadraticProjector:
    """Offline reference/oracle: argmin .5*(v-u)' W (v-u), A v<=b, SPD W.

    This Python enumerator is not a real-time chassis implementation. Runtime
    measurements do not establish a hardware or MPPI rollout deadline.

    In three dimensions an optimal normal cone needs at most three independent
    active rows. Enumerate these faces, solve with QR (no normal equations),
    require primal feasibility and nonnegative multipliers, and select minimum
    cost. Precomputation is separated from the per-command work.
    """

    def __init__(self, model, metric=None):
        self.model = model
        self.W = np.eye(3) if metric is None else array(metric, (3, 3), "projection metric")
        if not np.allclose(self.W, self.W.T, atol=1e-12, rtol=1e-12):
            raise ValueError("projection metric must be symmetric positive definite")
        try:
            self.L = np.linalg.cholesky(self.W)
        except np.linalg.LinAlgError as error:
            raise ValueError("projection metric must be symmetric positive definite") from error
        if np.linalg.cond(self.W) > 1e12:
            raise ValueError("projection metric is numerically ill-conditioned")
        self.inverse_lt = np.linalg.solve(self.L.T, np.eye(3))
        D = model.A @ self.inverse_lt
        norm = np.linalg.norm(D, axis=1)
        self.D, self.rhs = D / norm[:, None], model.b / norm
        self.faces = []
        for size in (1, 2, 3):
            for index in combinations(range(len(D)), size):
                rows = self.D[list(index)]
                singular = np.linalg.svd(rows, compute_uv=False)
                if singular[-1] < 1e-11 * singular[0]:
                    continue
                Q, R = np.linalg.qr(rows.T, mode="reduced")
                constant = np.linalg.solve(R.T, self.rhs[list(index)])
                self.faces.append((Q, R, constant))

    def project(self, commands):
        source = vectors(commands, 3, "commands")
        shape = source.shape
        flat = source.reshape(-1, 3)
        output = flat.copy()
        pending = np.flatnonzero(~self.model.contains(flat, tolerance=1e-12))
        if not len(pending):
            return output.reshape(shape)
        x0 = flat[pending] @ self.L
        best_cost = np.full(len(x0), np.inf)
        best = np.zeros_like(x0)
        for Q, R, constant in self.faces:
            normal = x0 @ Q - constant
            lam = np.linalg.solve(R, normal.T).T
            cost = np.einsum("ij,ij->i", normal, normal)
            mask = (cost < best_cost) & np.all(lam >= -1e-10 * (1+np.linalg.norm(x0, axis=1))[:, None], axis=1)
            indices = np.flatnonzero(mask)
            if not len(indices):
                continue
            candidate = x0[indices] - normal[indices] @ Q.T
            feasible = np.all((candidate @ self.D.T - self.rhs) / self.rhs <= 1e-9, axis=1)
            indices = indices[feasible]
            best[indices] = candidate[feasible]
            best_cost[indices] = cost[indices]
        if not np.all(np.isfinite(best_cost)):
            raise OptimizationError("no numerically certified projection; no fallback was applied")
        output[pending] = best @ self.inverse_lt.T
        if not np.all(self.model.contains(output, tolerance=1e-8)):
            raise OptimizationError("projected command violates physical halfspaces")
        return output.reshape(shape)

    def optimality_residual(self, source, projected):
        """Projection variational inequality over all vertices, normalized by cost scale."""
        u = vectors(source, 3, "source").reshape(-1, 3)
        v = vectors(projected, 3, "projected").reshape(-1, 3)
        gradient = (v-u) @ self.W
        values = np.einsum("ij,ikj->ik", gradient, self.model.vertices[None, :, :] - v[:, None, :])
        return float(np.maximum(0., -values.min(axis=1) /
                               (1 + np.einsum("ij,ij->i", gradient, v-u))).max(initial=0.))


@dataclass
class LinearConstraintCandidate:
    matrix: np.ndarray
    bound: np.ndarray
    drive_normalizer: np.ndarray
    variable: str
    row_labels: list

    def relative_violation(self, values):
        z = vectors(values, self.matrix.shape[1], self.variable)
        return np.maximum(0., ((z @ self.matrix.T-self.bound) / self.drive_normalizer).max(axis=-1))

    def tinympc_arguments(self):
        """Alin_x, blin_x, Alin_u, blin_u; no import or call into TinyMPC."""
        return ((self.matrix, self.bound, None, None) if self.variable == "state" else
                (None, None, self.matrix, self.bound))

    def export(self):
        return dict(research_only=True, hardware_validated=False, variable=self.variable,
                    relation="matrix @ variable <= bound", matrix=self.matrix.tolist(),
                    bound=self.bound.tolist(), original_drive_bounds=self.drive_normalizer.tolist(),
                    row_labels=self.row_labels, frame="base_link")


def tinympc_candidate(model, variable="input", mapping=None, offset=None, margin=0.):
    """Affine body velocity of ONE variable group. Coupled unknown x/u is not separable."""
    if variable not in ("state", "input") or not 0 <= margin < 1:
        raise ValueError("state/input and explicit fractional margin in [0,1) required")
    matrix, bound = model.linear_constraints(mapping, offset)
    bound = bound - margin * model.b
    labels = [n+":upper" for n in model.names] + [n+":lower" for n in model.names]
    return LinearConstraintCandidate(matrix, bound, model.b.copy(), variable, labels)


class CommandProjector:
    """Offline MPPI research candidate: (...,3) body twists -> body twists.

    The caller must evaluate rollouts and costs using the changed commands.
    Quadratic methods use the Python reference/oracle, not a real-time backend.
    This class owns no command output, timing, dynamics, stop or fallback policy.
    """

    def __init__(self, model, method, metric=None, box=None):
        self.model, self.method = model, method
        if method == "naive_axis":
            self.box = model.axis_box
        elif method == "conservative_box":
            self.box = model.inner_box
        elif method == "maximum_box":
            self.box = box if box is not None else maximum_box(model)["box"]
            lo = array(self.box.lower, (3,), "precomputed box lower")
            hi = array(self.box.upper, (3,), "precomputed box upper")
            if np.any(lo >= hi) or not np.all(model.contains(self.box.corners, tolerance=1e-8)):
                raise ValueError("precomputed box must be nonempty and wheel-feasible")
            if not np.all(self.box.contains(np.zeros(3))):
                raise ValueError("MPPI box candidate must contain the zero command")
        elif method not in ("ray", "euclidean", "weighted"):
            raise ValueError("unknown projection method")
        if method == "weighted" and metric is None:
            raise ValueError("weighted projection requires an explicit SPD metric")
        self.quadratic = QuadraticProjector(model, metric if method == "weighted" else None) if method in ("euclidean", "weighted") else None

    def project(self, commands):
        commands = vectors(commands, 3, "commands")
        if self.method in ("naive_axis", "conservative_box", "maximum_box"):
            return np.clip(commands, self.box.lower, self.box.upper)
        if self.method == "ray":
            return self.model.radial_clip(commands)
        return self.quadratic.project(commands)
