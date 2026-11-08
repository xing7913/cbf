"""Fast dependency-light solver for convex 2-D strictly convex QPs.

Solve min 0.5 * x.T @ Q @ x subject to G @ x <= h.
For a two-dimensional positive-definite diagonal Q, the optimum is either:
- the unconstrained minimizer (the origin),
- the Q-metric projection onto one active constraint, or
- an intersection of two active constraint boundaries.
All candidates and feasibility checks are vectorized with NumPy. This avoids
an external QP runtime dependency and is suitable for the ~200 CBF rows used
by this project.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


def solve_2d_qp(
    q_diag: tuple[float, float] | np.ndarray,
    g_matrix: np.ndarray,
    h_vector: np.ndarray,
    tolerance: float = 1e-8,
) -> Optional[np.ndarray]:
    """Return the feasible minimizer for a 2-D diagonal-Q QP, else None.

    The constraints are interpreted as G @ x <= h. The feasible region should
    be bounded by the caller's velocity limits; degenerate parallel boundaries
    are handled by skipping singular intersections.
    """
    q = np.asarray(q_diag, dtype=float).reshape(2)
    g = np.asarray(g_matrix, dtype=float)
    h = np.asarray(h_vector, dtype=float).reshape(-1)

    if g.ndim != 2 or g.shape[1] != 2 or g.shape[0] != h.size:
        raise ValueError("Expected G shape (m, 2) and h shape (m,)")
    if not np.all(np.isfinite(q)) or np.any(q <= 0.0):
        raise ValueError("Q diagonal entries must be finite and positive")
    if not np.all(np.isfinite(g)) or not np.all(np.isfinite(h)):
        return None

    q_inv = 1.0 / q
    candidates = [np.zeros((2, 1), dtype=float)]

    # Q-metric projections of the origin onto each constraint boundary.
    denom = np.sum((g * g) * q_inv[None, :], axis=1)
    valid = denom > tolerance
    if np.any(valid):
        projections = (q_inv[None, :] * g[valid]) * (h[valid] / denom[valid])[:, None]
        candidates.append(projections.T)

    # Vectorized intersections of all independent pairs of boundary lines.
    m = g.shape[0]
    if m >= 2:
        ii, jj = np.triu_indices(m, k=1)
        a, b = g[ii, 0], g[ii, 1]
        c, d = g[jj, 0], g[jj, 1]
        det = a * d - b * c
        independent = np.abs(det) > tolerance
        if np.any(independent):
            hi, hj = h[ii[independent]], h[jj[independent]]
            det = det[independent]
            x = (hi * d[independent] - b[independent] * hj) / det
            y = (a[independent] * hj - hi * c[independent]) / det
            candidates.append(np.vstack((x, y)))

    all_candidates = np.hstack(candidates)
    lhs = g @ all_candidates
    feasible = np.all(lhs <= h[:, None] + tolerance, axis=0)
    if not np.any(feasible):
        return None

    feasible_candidates = all_candidates[:, feasible]
    costs = 0.5 * np.sum(q[:, None] * feasible_candidates**2, axis=0)
    return feasible_candidates[:, int(np.argmin(costs))].copy()
