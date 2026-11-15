"""PID controller with derivative-on-measurement and anti-windup."""

from dataclasses import dataclass
import math


@dataclass
class PidGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    # Cap on the integral contribution (same units as the output).
    i_limit: float = math.inf
    # Symmetric output limit.
    out_limit: float = math.inf


class Pid:
    """Single-axis PID.

    * The derivative acts on the measured rate rather than on the error, so a
      setpoint step does not produce a derivative kick. The caller passes the
      rate directly (e.g. DVL-derived velocity), which is far less noisy than
      differentiating a position estimate.
    * Anti-windup is two-fold: the integral term is clamped to ``i_limit``,
      and integration is frozen whenever the output is saturated in the
      same direction as the error (conditional integration).
    """

    def __init__(self, gains: PidGains):
        self.gains = gains
        self.integral = 0.0  # stored as the integral *contribution*
        self.saturated = False

    def reset(self, integral: float = 0.0) -> None:
        self.integral = integral
        self.saturated = False

    def update(self, error: float, rate: float, dt: float,
               feedforward: float = 0.0) -> float:
        g = self.gains
        p_term = g.kp * error
        d_term = -g.kd * rate
        unclamped = p_term + self.integral + d_term + feedforward
        pushing_further = (unclamped > g.out_limit and error > 0) or \
                          (unclamped < -g.out_limit and error < 0)
        if dt > 0.0 and g.ki != 0.0 and not pushing_further:
            self.integral += g.ki * error * dt
            self.integral = max(-g.i_limit, min(g.i_limit, self.integral))
        out = p_term + self.integral + d_term + feedforward
        clamped = max(-g.out_limit, min(g.out_limit, out))
        self.saturated = clamped != out
        return clamped
