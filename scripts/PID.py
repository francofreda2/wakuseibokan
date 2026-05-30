"""
PID.py — controlador PID generico reutilizable.

Implementa los tres terminos clasicos:
    u(t) = Kp * e(t) + Ki * integral(e) + Kd * de/dt

Features incluidas:
    - anti-windup: el termino integral se clampa para no diverger cuando la
      salida ya esta saturada
    - clamping de la salida
    - filtrado pasa-bajos del termino derivativo (Td * D / (Td + dt)) para
      no amplificar ruido alto en la senal de entrada
    - manejo de wrap-around angular (heading_error) opcional

Convencion: error = setpoint - process_variable.
    error > 0 => salida positiva (sumando) tipica
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


def normalize_angle_deg(a: float) -> float:
    """Normaliza un angulo en grados a (-180, 180]."""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


@dataclass
class PIDController:
    """PID con anti-windup y clamping de salida.

    Parametros (todos opcionales menos kp):
        kp, ki, kd          ganancias proporcional, integral, derivativo
        output_min/max      saturacion (clamp) de la salida
        integral_min/max    saturacion del termino integral (anti-windup)
        derivative_alpha    filtro EMA en derivativo (0..1, 1 = sin filtro)
        angular             True si el error es angular en grados,
                            se aplica normalize_angle_deg
    """

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = -1.0
    output_max: float = 1.0
    integral_min: float = -1.0
    integral_max: float = 1.0
    derivative_alpha: float = 0.5
    angular: bool = False

    _integral: float = field(default=0.0, init=False)
    _last_error: float | None = field(default=None, init=False)
    _last_derivative: float = field(default=0.0, init=False)
    last_p: float = field(default=0.0, init=False)
    last_i: float = field(default=0.0, init=False)
    last_d: float = field(default=0.0, init=False)
    last_output: float = field(default=0.0, init=False)

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = None
        self._last_derivative = 0.0
        self.last_p = self.last_i = self.last_d = 0.0
        self.last_output = 0.0

    def step(self, setpoint: float, pv: float, dt: float) -> float:
        error = setpoint - pv
        if self.angular:
            error = normalize_angle_deg(error)

        # termino integral con anti-windup (clamp)
        self._integral += error * dt
        if self._integral > self.integral_max:
            self._integral = self.integral_max
        elif self._integral < self.integral_min:
            self._integral = self.integral_min

        # termino derivativo con filtro EMA
        if self._last_error is None or dt <= 0:
            derivative_raw = 0.0
        else:
            de = error - self._last_error
            if self.angular:
                de = normalize_angle_deg(de)
            derivative_raw = de / dt
        derivative = (self.derivative_alpha * derivative_raw
                      + (1 - self.derivative_alpha) * self._last_derivative)

        # salida
        p_term = self.kp * error
        i_term = self.ki * self._integral
        d_term = self.kd * derivative
        u = p_term + i_term + d_term

        # clamp salida y anti-windup back-calculation:
        # si la salida fue saturada, descontar la parte del integral que
        # excedio la saturacion para no acumular indefinidamente
        u_clamped = max(self.output_min, min(self.output_max, u))
        if u != u_clamped and self.ki != 0:
            excess = (u - u_clamped) / self.ki
            self._integral -= excess
            # re-clamp en caso de que la correccion lo saque del rango integral
            self._integral = max(self.integral_min,
                                 min(self.integral_max, self._integral))

        self._last_error = error
        self._last_derivative = derivative
        self.last_p, self.last_i, self.last_d = p_term, i_term, d_term
        self.last_output = u_clamped
        return u_clamped


# ---------------------------------------------------------------------------
# Self-test: respuesta al escalon
# ---------------------------------------------------------------------------

def _self_test() -> None:
    """Simula una planta de primer orden y prueba el PID.

    Planta: y' = (u - y) / tau    con tau = 0.5 s
    Setpoint: escalon a 1.0 a t=0
    """
    pid = PIDController(kp=2.0, ki=0.8, kd=0.3,
                        output_min=-5.0, output_max=5.0,
                        integral_min=-3.0, integral_max=3.0)

    y = 0.0
    dt = 0.05
    tau = 0.5
    setpoint = 1.0

    print(f"{'t':>5} {'sp':>6} {'pv':>7} {'u':>7} {'P':>7} {'I':>7} {'D':>7}")
    for k in range(0, 100):
        t = k * dt
        u = pid.step(setpoint, y, dt)
        y += (u - y) / tau * dt
        if k % 5 == 0:
            print(f"{t:5.2f} {setpoint:6.3f} {y:7.3f} {u:7.3f}"
                  f" {pid.last_p:7.3f} {pid.last_i:7.3f} {pid.last_d:7.3f}")
    print(f"\nError final: {setpoint - y:+.4f} (deberia tender a 0 con I)")

    print("\n-- Test wrap angular --")
    pid_ang = PIDController(kp=0.05, ki=0.001, kd=0.02,
                            output_min=-1.0, output_max=1.0,
                            angular=True)
    sp = 170.0
    pv = -170.0    # error real = -20 deg (no 340 deg)
    out = pid_ang.step(sp, pv, dt=0.05)
    print(f"setpoint=170, pv=-170 => error normalizado={pid_ang._last_error:.1f} deg, u={out:.3f}")


if __name__ == '__main__':
    _self_test()
