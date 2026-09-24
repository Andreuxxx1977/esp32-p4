"""4-layer HDI stack-up and closed-form controlled-impedance geometry.

The geometry is solved with Hammerstad-Jensen (surface microstrip, with the
Wheeler/Hammerstad thickness correction) and the IPC-2141A edge-coupled
microstrip relation. Treat the results as the *starting* geometry: send the
stack-up to PCBWay for their field-solver (Polar SI9000) confirmation and let
them adjust widths by +/-10 um -- that is normal practice for a first build.
Solder mask (LPI, ~15 um over the trace) typically lowers Z by 1-3 ohm; the
+/-10 % tolerance absorbs it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# Stack-up: 1.6 mm, 4 layers, 1+2+1 HDI (L1-L2 and L4-L3 laser microvias).
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Layer:
    name: str
    kind: str          # "copper" | "dielectric"
    thickness_mm: float
    material: str = ""
    er: float = 0.0
    function: str = ""


STACKUP: tuple[Layer, ...] = (
    Layer("F.Cu", "copper", 0.035, "Cu 0.5 oz + plating", function="Signal_Top (HS routing, refs L2 GND)"),
    Layer("PP1", "dielectric", 0.100, "Prepreg 1080 (laser-drillable)", er=4.0),
    Layer("In1.Cu", "copper", 0.035, "Cu 1 oz", function="GND (solid, un-split reference plane)"),
    Layer("Core", "dielectric", 1.260, "FR-4 core, Tg170", er=4.4),
    Layer("In2.Cu", "copper", 0.035, "Cu 1 oz", function="VCC_3V3 plane (+VDD_HP island under U1)"),
    Layer("PP2", "dielectric", 0.100, "Prepreg 1080 (laser-drillable)", er=4.0),
    Layer("B.Cu", "copper", 0.035, "Cu 0.5 oz + plating", function="Signal_Bottom (low-speed, heat-spreader pour)"),
)

BOARD_THICKNESS_MM = round(sum(layer.thickness_mm for layer in STACKUP), 3)

H_OUTER_MM = 0.100   # L1->L2 (and L4->L3) dielectric height
ER_OUTER = 4.0
T_CU_MM = 0.035


def _eeff(w: float, h: float, er: float) -> float:
    u = w / h
    return (er + 1) / 2 + (er - 1) / 2 * (1 + 12 / u) ** -0.5


def microstrip_z0(w: float, h: float = H_OUTER_MM, er: float = ER_OUTER,
                  t: float = T_CU_MM) -> float:
    """Single-ended surface-microstrip impedance (ohm)."""
    # thickness correction (effective width)
    if t > 0:
        w = w + (t / math.pi) * (1 + math.log(4 * math.e / math.sqrt(
            (t / h) ** 2 + (1 / math.pi / (w / t + 1.1)) ** 2)))
    u = w / h
    ee = _eeff(w, h, er)
    if u <= 1:
        return 60 / math.sqrt(ee) * math.log(8 / u + u / 4)
    return 120 * math.pi / (math.sqrt(ee) * (u + 1.393 + 0.667 * math.log(u + 1.444)))


def diff_microstrip_z(w: float, s: float, h: float = H_OUTER_MM,
                      er: float = ER_OUTER, t: float = T_CU_MM) -> float:
    """Edge-coupled surface-microstrip differential impedance (IPC-2141A)."""
    return 2 * microstrip_z0(w, h, er, t) * (1 - 0.48 * math.exp(-0.96 * s / h))


def _solve(fn, target: float, lo: float = 0.03, hi: float = 1.0) -> float:
    """Bisection on width (impedance falls monotonically with width)."""
    for _ in range(80):
        mid = (lo + hi) / 2
        if fn(mid) > target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def width_for_se(z_target: float) -> float:
    return _solve(lambda w: microstrip_z0(w), z_target)


def width_for_diff(z_target: float, gap_mm: float) -> float:
    return _solve(lambda w: diff_microstrip_z(w, gap_mm), z_target)


@dataclass(frozen=True)
class ImpedanceRule:
    netclass: str
    target_ohm: float
    kind: str              # "se" | "diff"
    width_mm: float
    gap_mm: float | None
    achieved_ohm: float
    nets_hint: str


def _round_um(x: float, step: float = 0.005) -> float:
    return round(round(x / step) * step, 3)


def impedance_rules() -> tuple[ImpedanceRule, ...]:
    """Final (rounded to 5 um) geometry for every controlled-impedance class."""
    out = []
    w50 = _round_um(width_for_se(50.0))
    out.append(ImpedanceRule("SE_50", 50.0, "se", w50, None,
                             round(microstrip_z0(w50), 1),
                             "SDIO, RMII, QSPI flash, crystal"))
    for cls, z, gap, hint in (
        ("USB_90", 90.0, 0.150, "USB 2.0 HS (J1) and FS (J2/J3) D+/D-"),
        ("MIPI_100", 100.0, 0.200, "MIPI CSI/DSI clock + data lanes"),
        ("ETH_100", 100.0, 0.200, "PHY <-> RJ45 magnetics TX/RX pairs"),
    ):
        w = _round_um(width_for_diff(z, gap))
        out.append(ImpedanceRule(cls, z, "diff", w, gap,
                                 round(diff_microstrip_z(w, gap), 1), hint))
    return tuple(out)


if __name__ == "__main__":  # pragma: no cover - manual inspection helper
    print(f"Board thickness: {BOARD_THICKNESS_MM} mm")
    for r in impedance_rules():
        gap = f"{r.gap_mm:.3f}" if r.gap_mm else "-"
        print(f"{r.netclass:9s} {r.target_ohm:5.0f} ohm  w={r.width_mm:.3f}  s={gap}  "
              f"-> {r.achieved_ohm} ohm  ({r.nets_hint})")
