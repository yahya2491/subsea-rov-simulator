"""Thruster allocation: body wrench <-> individual thruster forces.

Body frame is FLU (x forward, y left, z up) with its origin at the centre of
mass. Each thruster i has a mount position r_i and a unit thrust axis d_i; a
positive command pushes the vehicle along +d_i. Its contribution to the body
wrench is the column [d_i; r_i x d_i] of the 6 x n allocation matrix B.

Commands are found with the Moore-Penrose pseudo-inverse, which gives the
minimum-norm thrust vector for a requested wrench. Saturation is handled per
thruster group (e.g. horizontal / vertical): if any thruster in a group would
exceed its limit, the whole group is scaled down together. That keeps the
direction of the wrench each group produces, so a large surge demand cannot
leak into a yaw error, and a saturated horizontal group never steals heave
authority from the vertical group.
"""

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class Thruster:
    name: str
    position: Tuple[float, float, float]
    direction: Tuple[float, float, float]
    group: str = 'all'
    max_thrust: float = 15.0
    min_thrust: float = -15.0


def thrusters_from_arrays(
    names: Sequence[str],
    positions: Sequence[float],
    directions: Sequence[float],
    groups: Sequence[str],
    max_thrust: float,
    min_thrust: float,
) -> List[Thruster]:
    """Build thrusters from the flat parallel arrays used in ROS parameters."""
    n = len(names)
    if len(positions) != 3 * n or len(directions) != 3 * n or len(groups) != n:
        raise ValueError(
            f'thruster arrays disagree: {n} names, {len(positions)} position '
            f'values, {len(directions)} direction values, {len(groups)} groups')
    out = []
    for i, name in enumerate(names):
        d = np.asarray(directions[3 * i:3 * i + 3], dtype=float)
        norm = np.linalg.norm(d)
        if norm < 1e-9:
            raise ValueError(f'thruster {name} has a zero thrust direction')
        out.append(Thruster(
            name=name,
            position=tuple(float(v) for v in positions[3 * i:3 * i + 3]),
            direction=tuple(d / norm),
            group=groups[i],
            max_thrust=max_thrust,
            min_thrust=min_thrust,
        ))
    return out


class ThrusterAllocator:
    """Maps a 6-DOF body wrench to saturated thruster forces."""

    def __init__(self, thrusters: Sequence[Thruster]):
        if not thrusters:
            raise ValueError('at least one thruster is required')
        self.thrusters = list(thrusters)
        self.names = [t.name for t in self.thrusters]
        self.matrix = self._build_matrix(self.thrusters)
        rank = np.linalg.matrix_rank(self.matrix)
        if rank < 6:
            raise ValueError(f'allocation matrix has rank {rank}; '
                             'the layout cannot control all 6 DOF')
        self.pinv = np.linalg.pinv(self.matrix)
        self.max_thrust = np.array([t.max_thrust for t in self.thrusters])
        self.min_thrust = np.array([t.min_thrust for t in self.thrusters])
        self.groups: Dict[str, np.ndarray] = {}
        for i, t in enumerate(self.thrusters):
            self.groups.setdefault(t.group, []).append(i)
        self.groups = {g: np.array(idx) for g, idx in self.groups.items()}

    @staticmethod
    def _build_matrix(thrusters: Sequence[Thruster]) -> np.ndarray:
        cols = []
        for t in thrusters:
            r = np.asarray(t.position, dtype=float)
            d = np.asarray(t.direction, dtype=float)
            cols.append(np.concatenate([d, np.cross(r, d)]))
        return np.column_stack(cols)

    def allocate(self, wrench: Sequence[float]) -> Tuple[np.ndarray, Dict[str, float]]:
        """Return (thrust commands, per-group scale factor in (0, 1])."""
        tau = np.asarray(wrench, dtype=float).reshape(6)
        u = self.pinv @ tau
        scales = {}
        for group, idx in self.groups.items():
            scale = 1.0
            for i in idx:
                if u[i] > self.max_thrust[i]:
                    scale = min(scale, self.max_thrust[i] / u[i])
                elif u[i] < self.min_thrust[i]:
                    scale = min(scale, self.min_thrust[i] / u[i])
            u[idx] *= scale
            scales[group] = scale
        # Guard against round-off pushing a value a hair past its limit.
        u = np.clip(u, self.min_thrust, self.max_thrust)
        return u, scales

    def wrench_from_thrust(self, thrust: Sequence[float]) -> np.ndarray:
        """Body wrench produced by a thrust vector (forward map)."""
        return self.matrix @ np.asarray(thrust, dtype=float)

    def max_wrench(self) -> np.ndarray:
        """Largest single-axis wrench magnitude available on each DOF."""
        out = np.zeros(6)
        for axis in range(6):
            unit = np.zeros(6)
            unit[axis] = 1.0
            u = self.pinv @ unit
            # Largest k such that k*u stays inside every limit.
            with np.errstate(divide='ignore'):
                lim = np.where(u > 0, self.max_thrust / u,
                               np.where(u < 0, self.min_thrust / u, np.inf))
            out[axis] = float(np.min(lim))
        return out
