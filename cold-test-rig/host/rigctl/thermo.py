"""Thermocouple EMF <-> temperature, NIST ITS-90 reference functions (SRD 60).

Coefficients for types K and N copied from the `thermocouples_reference` package
(source_NIST.py, itself machine-built from NIST SRD 60), highest power first, emf in mV,
temperature in degC. Type K above 0 degC adds a0 * exp(a1 * (T - a2)^2).
Inverse is by Newton iteration on the forward function (no approximate inverse tables).
"""
from __future__ import annotations

import math

import numpy as np

_K = [
    (-270.0, 0.0, [-0.163226974860e-22, -0.198892668780e-19, -0.104516093650e-16, -0.310888728940e-14,
                   -0.574103274280e-12, -0.675090591730e-10, -0.499048287770e-08, -0.328589067840e-06,
                   0.236223735980e-04, 0.394501280250e-01, 0.000000000000e+00], None),
    (0.0, 1372.0, [-0.121047212750e-25, 0.971511471520e-22, -0.320207200030e-18, 0.560750590590e-15,
                   -0.560728448890e-12, 0.318409457190e-09, -0.994575928740e-07, 0.185587700320e-04,
                   0.389212049750e-01, -0.176004136860e-01], (0.118597600000e+00, -0.118343200000e-03, 0.126968600000e+03)),
]
_N = [
    (-270.0, 0.0, [-0.934196678350e-19, -0.760893007910e-16, -0.226534380030e-13, -0.263033577160e-11,
                   -0.464120397590e-10, -0.938411115540e-07, 0.109574842280e-04, 0.261591059620e-01,
                   0.000000000000e+00], None),
    (0.0, 1300.0, [-0.306821961510e-28, 0.208492293390e-24, -0.608632456070e-21, 0.997453389920e-18,
                   -0.100634715190e-14, 0.643118193390e-12, -0.252611697940e-09, 0.438256272370e-07,
                   0.157101418800e-04, 0.259293946010e-01, 0.000000000000e+00], None),
]
TYPES = {"K": _K, "N": _N}

ADS1115_UV_PER_LSB = 256000.0 / 32768.0   # +-0.256 V range: 7.8125 uV


def emf_mv(t_c: float, kind: str = "K") -> float:
    for lo, hi, c, g in TYPES[kind]:
        if lo <= t_c <= hi or (hi == TYPES[kind][-1][1] and t_c > hi):
            v = float(np.polyval(c, t_c))
            if g:
                v += g[0] * math.exp(g[1] * (t_c - g[2]) ** 2)
            return v
    raise ValueError(f"{t_c} degC outside the type {kind} range")


def temperature_c(emf_measured_mv: float, cj_c: float, kind: str = "K") -> float:
    """Hot-junction temperature from the measured EMF and the cold-junction temperature."""
    target = emf_measured_mv + emf_mv(cj_c, kind)
    t = cj_c + emf_measured_mv / 0.040
    for _ in range(30):
        f = emf_mv(t, kind) - target
        d = (emf_mv(t + 0.01, kind) - emf_mv(t - 0.01, kind)) / 0.02
        step = f / d
        t -= step
        if abs(step) < 1e-5:
            break
    return t


def from_ads1115(raw: int | None, cj_c: float | None, kind: str = "K") -> float | None:
    """Temperature from an ADS1115 raw count (PGA +-0.256 V). None if missing or open."""
    if raw is None or cj_c is None or raw >= 32767 or raw <= -32768:
        return None
    return temperature_c(raw * ADS1115_UV_PER_LSB / 1000.0, cj_c, kind)
