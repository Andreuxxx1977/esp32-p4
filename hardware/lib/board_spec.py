"""ESP32-P4 "Extreme Performance" board -- the single source of truth.

Everything downstream is generated from this module:

* ``hardware/skidl/esp32p4_extreme_netlist.py``  -> KiCad netlist (SKiDL)
* ``hardware/pcbnew/esp32p4_extreme_place.py``  -> board placement (pcbnew API)
* ``tools/gen_docs.py``                          -> pinout table + PCBWay BOM
* ``tests/``                                     -> conflict / keep-out checks

Coordinates are millimetres relative to the ESP32-P4 body centre, which is
the design origin (0, 0). Axes follow KiCad: +X right, +Y **down**. The four
M2.5 holes are symmetric about the origin, so the requested set
[-15, 15], [15, 15], [-15, -15], [15, -15] is identical in either Y convention.

Pin numbers of third-party parts were taken from the official KiCad symbol
library (gitlab.com/kicad/libraries/kicad-symbols) unless ``pin_source`` says
otherwise.
"""

from __future__ import annotations

import itertools
import re
from collections import Counter, defaultdict
from dataclasses import dataclass, field

from . import esp32p4_pinout as p4

# ===========================================================================
# 1. Data model
# ===========================================================================


@dataclass(frozen=True)
class PartType:
    """An orderable part: MPN + footprint + pin list."""

    key: str
    value: str
    mpn: str
    manufacturer: str
    footprint: str                          # "Library:Footprint" (KiCad)
    package: str                            # human readable, used in the BOM
    description: str
    pins: tuple[tuple[str, str], ...]       # (pad number, pin name)
    height_mm: float
    kind: str = "ic"                        # ic|res|cap|ind|fb|diode|led|conn|xtal|sw|fet|mech
    lcsc: str = ""
    tht: bool = False
    pin_source: str = "KiCad official symbol library"

    @property
    def pad_numbers(self) -> set[str]:
        return {n for n, _ in self.pins}

    def pad(self, name_or_number: str) -> str:
        """Resolve a pin *name* (or a pad number) to a pad number."""
        if name_or_number in self.pad_numbers:
            return name_or_number
        hits = [n for n, nm in self.pins if nm == name_or_number]
        if len(hits) != 1:
            raise KeyError(f"{self.key}: pin '{name_or_number}' matches {hits}")
        return hits[0]


@dataclass(frozen=True)
class Place:
    """Absolute placement (design coordinates, mm, degrees)."""

    x: float
    y: float
    rot: float = 0.0
    side: str = "F"                   # F = top, B = bottom
    faces: str | None = None          # edge connectors: 'left'|'right'|'top'|'bottom'


@dataclass(frozen=True)
class Near:
    """Relative placement: put the part next to ``ref`` (optionally a pad).

    ``max_mm`` is the maximum pad-to-pad distance the placer must honour
    (2.0 mm for SoC decoupling, per the hardware specification).
    """

    ref: str
    pad: str | None = None
    max_mm: float = 3.0


@dataclass
class Component:
    ref: str
    part: PartType
    conns: dict[str, str]             # pad number -> net name
    group: str
    value: str = ""
    dnp: bool = False
    place: Place | Near | None = None
    note: str = ""

    def __post_init__(self) -> None:
        if not self.value:
            self.value = self.part.value


# ===========================================================================
# 2. Part library (MPN + footprint + pins)
# ===========================================================================

def _pins(*names: str, start: int = 1) -> tuple[tuple[str, str], ...]:
    return tuple((str(i), n) for i, n in enumerate(names, start))


PARTS: dict[str, PartType] = {}


def _add(pt: PartType) -> PartType:
    assert pt.key not in PARTS, pt.key
    PARTS[pt.key] = pt
    return pt


ESP32P4 = _add(PartType(
    "ESP32-P4NRW32X", "ESP32-P4NRW32X", "ESP32-P4NRW32X", "Espressif Systems",
    "PCM_Espressif:ESP32-P4", "QFN-104 10x10 mm, 0.35 mm pitch, EPAD 7.5 mm",
    "Dual-core RISC-V 400 MHz SoC, 32 MB in-package PSRAM, chip rev v3.x",
    tuple((n, nm) for n, nm, _ in p4.PADS), p4.HEIGHT_MAX_MM, lcsc="C54540373",
    pin_source="Espressif KiCad library, symbol ESP32-P4X"))

FLASH = _add(PartType(
    "W25Q256JVEIQ", "W25Q256JV 32MB", "W25Q256JVEIQ", "Winbond",
    "Package_SON:WSON-8-1EP_8x6mm_P1.27mm_EP3.4x4.3mm", "WSON-8 8x6 mm",
    "256 Mbit (32 MB) 3.3 V Quad-SPI NOR flash, 133 MHz, 4-byte addressing",
    _pins("~CS", "DO/IO1", "~WP/IO2", "GND", "DI/IO0", "CLK", "~HOLD/IO3", "VCC", "EP"),
    0.80, lcsc="C97522",
    pin_source="KiCad W25Q32JVZP/W25Q128JVE (WSON-8 pin-compatible family)"))

BUCK_3V3 = _add(PartType(
    "TPS62130RGTR", "TPS62130", "TPS62130RGTR", "Texas Instruments",
    "Package_DFN_QFN:VQFN-16-1EP_3x3mm_P0.5mm_EP1.68x1.68mm_ThermalVias", "VQFN-16 3x3 mm",
    "3-17 V in, 3 A synchronous buck (DCS-Control), 100 % mode, 5 V->3V3 main rail",
    (("1", "SW"), ("2", "SW"), ("3", "SW"), ("4", "PG"), ("5", "FB"), ("6", "AGND"),
     ("7", "FSW"), ("8", "DEF"), ("9", "SS/TR"), ("10", "AVIN"), ("11", "PVIN"),
     ("12", "PVIN"), ("13", "EN"), ("14", "VOS"), ("15", "PGND"), ("16", "PGND"),
     ("17", "EP")), 1.00, lcsc="C43590"))

BUCK_HP = _add(PartType(
    "TLV62569DBVR", "TLV62569", "TLV62569DBVR", "Texas Instruments",
    "Package_TO_SOT_SMD:SOT-23-5", "SOT-23-5",
    "2 A buck, Espressif-verified external DCDC for ESP32-P4 VDD_HP (EN/FB driven by the SoC)",
    _pins("EN", "GND", "SW", "VIN", "FB"), 1.45, lcsc="C141836"))

PHY = _add(PartType(
    "LAN8720A-CP-TR", "LAN8720A", "LAN8720A-CP-TR", "Microchip",
    "Package_DFN_QFN:VQFN-24-1EP_4x4mm_P0.5mm_EP2.5x2.5mm_ThermalVias", "VQFN-24 4x4 mm",
    "10/100 Ethernet RMII PHY, HP Auto-MDIX, ESP-IDF 'lan87xx' driver",
    _pins("VDD2A", "LED2/~INTSEL", "LED1/REGOFF", "XTAL2", "XTAL1/CLKIN", "VDDCR",
          "RXD1/MODE1", "RXD0/MODE0", "VDDIO", "RXER/PHYAD0", "CRS_DV/MODE2", "MDIO",
          "MDC", "~INT/REFCLKO", "~RST", "TXEN", "TXD0", "TXD1", "VDD1A", "TXN", "TXP",
          "RXN", "RXP", "RBIAS", "VSS"), 0.90, lcsc="C45223"))

UART_BRIDGE = _add(PartType(
    "CP2102N-A02-GQFN24R", "CP2102N", "CP2102N-A02-GQFN24R", "Silicon Labs",
    "Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm", "QFN-24 4x4 mm",
    "USB 2.0 FS to UART bridge, 3 Mbaud, DTR/RTS for auto-program",
    _pins("~RI/CLK", "GND", "D+", "D-", "VIO", "VDD", "VREGIN", "VBUS", "~RST", "NC",
          "~WAKEUP/GPIO.3", "RS485/GPIO.2", "~RXT/GPIO.1", "~TXT/GPIO.0", "~SUSPEND",
          "NC", "SUSPEND", "~CTS", "~RTS", "RXD", "TXD", "~DSR", "~DTR", "~DCD", "GND"),
    0.90, lcsc="C969151"))

ESD_USB = _add(PartType(
    "USBLC6-2SC6", "USBLC6-2SC6", "USBLC6-2SC6", "STMicroelectronics",
    "Package_TO_SOT_SMD:SOT-23-6", "SOT-23-6",
    "2-line USB ESD with VBUS clamp (3.5 pF max: full-speed ports only)",
    _pins("I/O1", "GND", "I/O2", "I/O2", "VBUS", "I/O1"), 1.45, kind="diode", lcsc="C7519"))

ESD_USB_HS = _add(PartType(
    "TPD2EUSB30DRTR", "TPD2EUSB30", "TPD2EUSB30DRTR", "Texas Instruments",
    "Package_TO_SOT_SMD:Texas_DRT-3", "SOT-3 (DRT) 1.0x1.0 mm",
    "2-channel ultra-low-capacitance ESD (0.7 pF typ) for USB 2.0 HS -- Espressif limit is 1 pF",
    _pins("D+", "D-", "GND"), 0.60, kind="diode", lcsc="C97502"))

ESD_4CH = _add(PartType(
    "TPD4E05U06DQAR", "TPD4E05U06", "TPD4E05U06DQAR", "Texas Instruments",
    "Package_SON:USON-10_2.5x1.0mm_P0.5mm", "USON-10 2.5x1.0 mm",
    "4-channel 0.5 pF flow-through ESD array (microSD, UHS-I safe)",
    _pins("D1+", "D1-", "GND", "D2+", "D2-", "FT_D2-", "FT_D2+", "GND", "FT_D1-", "FT_D1+"), 0.60,
    kind="diode", lcsc="C138714",
    pin_source="KiCad TPD4E05U06DQA (extends TPD4EUSB30); pads 6/7/9/10 are the no-internal-"
               "connection flow-through partners of 5/4/2/1"))

DUAL_NPN = _add(PartType(
    "MMDT3904-7-F", "MMDT3904", "MMDT3904-7-F", "Diodes Incorporated",
    "Package_TO_SOT_SMD:SOT-363_SC-70-6", "SOT-363",
    "Dual NPN for DTR/RTS auto-reset + boot-strap circuit",
    _pins("E1", "B1", "C2", "E2", "B2", "C1"), 1.10, kind="fet", lcsc="C83572",
    pin_source="KiCad Q_Dual_NPN_NPN_E1B1C2E2B2C1"))

NMOS = _add(PartType(
    "AO3400A", "AO3400A", "AO3400A", "Alpha & Omega Semiconductor",
    "Package_TO_SOT_SMD:SOT-23", "SOT-23",
    "30 V 5.7 A logic-level N-MOSFET (Vgs(th) 1.45 V max)",
    _pins("G", "S", "D"), 1.25, kind="fet", lcsc="C20917"))

PMOS = _add(PartType(
    "AO3401A", "AO3401A", "AO3401A", "Alpha & Omega Semiconductor",
    "Package_TO_SOT_SMD:SOT-23", "SOT-23",
    "-30 V -4 A P-MOSFET (high-side load switch)",
    _pins("G", "S", "D"), 1.25, kind="fet", lcsc="C15127"))

USB_C = _add(PartType(
    "USB4105-GF-A", "USB-C", "USB4105-GF-A", "GCT",
    "Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal",
    "USB Type-C 16P, top-mount", "USB 2.0 Type-C receptacle (UFP, 5.1k Rd); SMD contacts + 4 THT shell legs",
    (("A1", "GND"), ("A4", "VBUS"), ("A5", "CC1"), ("A6", "D+"), ("A7", "D-"),
     ("A8", "SBU1"), ("A9", "VBUS"), ("A12", "GND"), ("B1", "GND"), ("B4", "VBUS"),
     ("B5", "CC2"), ("B6", "D+"), ("B7", "D-"), ("B8", "SBU2"), ("B9", "VBUS"),
     ("B12", "GND"), ("SH", "SHIELD")), 3.31, kind="conn", lcsc="C3020560",
    pin_source="KiCad USB_C_Receptacle_USB2.0_16P; GCT footprint shell = 4 plated 'SH' legs (pin-in-paste)"))

MICROSD = _add(PartType(
    "DM3AT-SF-PEJM5", "microSD", "DM3AT-SF-PEJM5", "Hirose",
    "Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5", "microSD push-push, SMT",
    "microSD socket with card-detect switch",
    (("1", "DAT2"), ("2", "DAT3/CD"), ("3", "CMD"), ("4", "VDD"), ("5", "CLK"),
     ("6", "VSS"), ("7", "DAT0"), ("8", "DAT1"), ("9", "DET_B"), ("10", "DET_A"),
     ("SH", "SHIELD")), 1.98, kind="conn", lcsc="C114218",
    pin_source="KiCad Micro_SD_Card_Det2; footprint pads 1-10 + 4 'SH' shield pads"))

# Raspberry Pi 22-pin 0.5 mm MIPI pinout (Pi 5 / CM4 / Zero camera & display).
RPI22_PINS = _pins(
    "GND", "D0_N", "D0_P", "GND", "D1_N", "D1_P", "GND", "CLK_N", "CLK_P", "GND",
    "D2_N", "D2_P", "GND", "D3_N", "D3_P", "GND", "IO0", "IO1", "GND", "SCL", "SDA",
    "3V3") + (("MP", "MP"),)

FPC22 = _add(PartType(
    "F32Q-1A7H1-11022", "FPC 22P 0.5mm", "F32Q-1A7H1-11022", "Amphenol ICC",
    "Connector_FFC-FPC:Amphenol_F32Q-1A7x1-11022_1x22-1MP_P0.5mm_Horizontal",
    "FPC 22P 0.5 mm, top contact, horizontal, SMT",
    "22-pin 0.5 mm FPC, same top-contact part as Raspberry Pi 5 -> standard Pi cables, no mirroring",
    RPI22_PINS, 2.00, kind="conn", lcsc="C3169253",
    pin_source="raspberrypi/documentation accessories/camera (22-pin CSI); Pi 5 DSI uses the same pin positions"))

RJ45 = _add(PartType(
    "HR911105A", "RJ45 MagJack", "HR911105A", "HanRun",
    "Connector_RJ:RJ45_Hanrun_HR911105A_Horizontal", "RJ45 THT, integrated magnetics + 2 LEDs",
    "RJ45 with 10/100 magnetics (1:1, CT) and green/yellow LEDs",
    (("1", "TD+"), ("2", "TD-"), ("3", "RD+"), ("4", "TCT"), ("5", "RCT"), ("6", "RD-"),
     ("7", "NC"), ("8", "GND"), ("9", "LED_G_A"), ("10", "LED_G_K"), ("11", "LED_Y_K"),
     ("12", "LED_Y_A"), ("SH", "SHIELD")), 13.3, kind="conn", tht=True, lcsc="C12074",
    pin_source="KiCad RJ45_Hanrun_HR911105A_Horizontal (pins 1-8); LED pins 9-12 per two independent libraries"))

FAN_HDR = _add(PartType(
    "47053-1000", "Fan 4P", "47053-1000", "Molex",
    "Connector:FanPinHeader_1x04_P2.54mm_Vertical", "2.54 mm 4-pin fan header, THT",
    "Intel 4-wire PWM fan header (1 GND, 2 +V, 3 TACH, 4 PWM)",
    _pins("GND", "+5V", "TACH", "PWM"), 11.4, kind="conn", tht=True, lcsc="C240840",
    pin_source="Intel 4-wire fan specification"))

HDR_2X12 = _add(PartType(
    "61302421121", "2x12 header", "61302421121", "Wurth Elektronik",
    "Connector_PinHeader_2.54mm:PinHeader_2x12_P2.54mm_Vertical", "2x12 2.54 mm THT",
    "2x12 expansion header", tuple((str(i), f"P{i}") for i in range(1, 25)), 8.5,
    kind="conn", tht=True, pin_source="Conn_02x12_Odd_Even"))

HDR_2X8 = _add(PartType(
    "61301621121", "2x8 header", "61301621121", "Wurth Elektronik",
    "Connector_PinHeader_2.54mm:PinHeader_2x08_P2.54mm_Vertical", "2x8 2.54 mm THT",
    "2x8 SPI/I2C TFT display header (dupont-wire friendly)",
    tuple((str(i), f"P{i}") for i in range(1, 17)), 8.5, kind="conn", tht=True,
    pin_source="Conn_02x08_Odd_Even"))

JST_SH2 = _add(PartType(
    "SM02B-SRSS-TB(LF)(SN)", "JST-SH 2P", "SM02B-SRSS-TB(LF)(SN)", "JST",
    "Connector_JST:JST_SH_SM02B-SRSS-TB_1x02-1MP_P1.00mm_Horizontal", "JST SH 1.0 mm 2P, SMT",
    "Display backlight 5 V feed", (("1", "1"), ("2", "2"), ("MP", "MP")), 2.95, kind="conn",
    lcsc="C160402"))

XTAL40 = _add(PartType(
    "402F4001XIAR", "40MHz 10ppm", "402F4001XIAR", "CTS",
    "Crystal:Crystal_SMD_2016-4Pin_2.0x1.6mm", "2016 4-pad",
    "40 MHz +/-10 ppm crystal, C_L 10 pF, ESR 60 ohm max, for ESP32-P4",
    _pins("XIN", "GND", "XOUT", "GND"), 0.50, kind="xtal", lcsc="C5508101",
    pin_source="KiCad Crystal_GND24"))

XTAL25 = _add(PartType(
    "X322525MOB4SI", "25MHz", "X322525MOB4SI", "YXC",
    "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm", "3225 4-pad",
    "25 MHz +/-10 ppm / +/-20 ppm stab., C_L 12 pF -- inside the +/-50 ppm 100BASE-TX budget",
    _pins("XIN", "GND", "XOUT", "GND"), 0.80, kind="xtal", lcsc="C9006",
    pin_source="KiCad Crystal_GND24"))

SCHOTTKY = _add(PartType(
    "PMEG4050EP,115", "PMEG4050EP", "PMEG4050EP,115", "Nexperia",
    "Diode_SMD:D_SOD-128", "SOD-128",
    "40 V 5 A low-VF Schottky (~0.4 V @ 2.5 A) -- VBUS OR-ing", _pins("K", "A"), 1.10,
    kind="diode", lcsc="C96235"))

PTC = _add(PartType(
    "MF-MSMF250/16X-2", "PTC 2.5A", "MF-MSMF250/16X-2", "Bourns",
    "Fuse:Fuse_1812_4532Metric", "1812", "Resettable fuse, 2.5 A hold / 5 A trip, 16 V, on VSYS",
    _pins("1", "2"), 1.10, kind="fb", lcsc="C210838"))

FERRITE = _add(PartType(
    "BLM18PG221SN1D", "220R@100MHz", "BLM18PG221SN1D", "Murata",
    "Inductor_SMD:L_0603_1608Metric", "0603", "Ferrite bead 220 ohm, 1.4 A",
    _pins("1", "2"), 0.80, kind="fb", lcsc="C80165"))

IND_3V3 = _add(PartType(
    "XAL5030-222MEC", "2.2uH", "XAL5030-222MEC", "Coilcraft",
    "Inductor_SMD:L_Coilcraft_XAL5030-XXX", "5.3x5.5 mm shielded",
    "2.2 uH, Isat 9.2 A, DCR 14.5 mOhm -- 3 A buck inductor",
    _pins("1", "2"), 3.10, kind="ind", lcsc="C920280"))

IND_HP = _add(PartType(
    "DFE252012F-2R2M=P2", "2.2uH", "DFE252012F-2R2M=P2", "Murata",
    "Inductor_SMD:L_1008_2520Metric", "2520 (1008) metal alloy",
    "2.2 uH, Isat 3.3 A, 82 mOhm -- VDD_HP buck inductor",
    _pins("1", "2"), 1.20, kind="ind", lcsc="C576403",
    pin_source="KiCad has no DFE252012F footprint: generic 1008/2520 land -- check Murata land drawing"))

TACT = _add(PartType(
    "PTS810SJM250SMTRLFS", "Tactile", "PTS810SJM250SMTRLFS", "C&K",
    "Button_Switch_SMD:SW_SPST_PTS810", "4.2x3.2 mm SMD", "SMD tactile switch",
    _pins("1", "2"), 2.50, kind="sw",
    lcsc="C116501",
    pin_source="KiCad SW_SPST_PTS810 footprint: 2 terminals, each on two pads ('1','1','2','2')"))

LED_G = _add(PartType(
    "150060GS75000", "Green", "150060GS75000", "Wurth Elektronik",
    "LED_SMD:LED_0603_1608Metric", "0603", "Green LED", _pins("K", "A"), 0.80, kind="led",
    lcsc="C5252984"))

LED_R = _add(PartType(
    "150060RS75000", "Red", "150060RS75000", "Wurth Elektronik",
    "LED_SMD:LED_0603_1608Metric", "0603", "Red LED", _pins("K", "A"), 0.80, kind="led",
    lcsc="C3030991"))

MH_M25 = _add(PartType(
    "MH-M2.5", "M2.5", "", "", "MountingHole:MountingHole_2.7mm_M2.5_Pad_Via",
    "plated M2.5 hole", "Heatsink/fan clamp hole (GND)", (("1", "1"),), 0.0, kind="mech"))

MH_M3 = _add(PartType(
    "MH-M3", "M3", "", "", "MountingHole:MountingHole_3.2mm_M3_Pad_Via",
    "plated M3 hole", "Board mounting hole (GND)", (("1", "1"),), 0.0, kind="mech"))

# --- passives --------------------------------------------------------------
_RES_CODE = {
    "0": None, "22": "22R", "33": "33R", "49.9": "49R9", "100": "100R", "330": "330R",
    "1k": "1K", "1.5k": "1K5", "2.2k": "2K2", "4.02k": "4K02", "5.1k": "5K1",
    "10k": "10K", "12.1k": "12K1", "22.1k": "22K1", "24k": "24K", "47.5k": "47K5",
    "51k": "51K", "75k": "75K", "100k": "100K", "499k": "499K", "1M": "1M",
}

_CAP_MPN = {
    # (value, size): (mpn, rating/dielectric, height)
    ("10pF", "0402"): ("CL05C100CB5NNNC", "50V C0G", 0.55),
    ("12pF", "0402"): ("CL05C120JB5NNNC", "50V C0G", 0.55),
    ("18pF", "0402"): ("CL05C180JB5NNNC", "50V C0G", 0.55),
    ("22pF", "0402"): ("CL05C220JB5NNNC", "50V C0G", 0.55),
    ("100pF", "0402"): ("CL05C101JB5NNNC", "50V C0G", 0.55),
    ("470pF", "0402"): ("CL05B471KB5NNNC", "50V X7R", 0.55),
    ("3.3nF", "0402"): ("CL05B332KB5NNNC", "50V X7R", 0.55),
    ("10nF", "0402"): ("CL05B103KB5NNNC", "50V X7R", 0.55),
    ("100nF", "0402"): ("CL05B104KO5NNNC", "16V X7R", 0.55),
    ("1uF", "0402"): ("CL05A105KA5NQNC", "25V X5R", 0.55),
    ("4.7uF", "0402"): ("CL05A475MP5NRNC", "10V X5R", 0.55),
    ("10uF", "0402"): ("CL05A106MQ5NUNC", "6.3V X5R", 0.55),
    ("10uF", "0805"): ("CL21A106KAYNNNE", "25V X5R", 1.45),
    ("22uF", "0805"): ("CL21A226MQQNNNE", "6.3V X5R", 1.45),
    ("47uF", "0805"): ("CL21A476MQYNNNE", "6.3V X5R", 1.45),
}

_METRIC = {"0402": "1005", "0603": "1608", "0805": "2012"}

# LCSC numbers confirmed by the MPN verification pass (blank = not found on LCSC).
_LCSC = {
    "CL05B104KO5NNNC": "C1525", "CL05B103KB5NNNC": "C15195", "CL05A105KA5NQNC": "C52923",
    "CL05A106MQ5NUNC": "C15525", "CL05A475MP5NRNC": "C23733", "CL05C220JB5NNNC": "C70464",
    "CL05B471KB5NNNC": "C26412", "CL05B332KB5NNNC": "C26404", "CL21A226MQQNNNE": "C5674",
    "CL21A106KAYNNNE": "C15850", "CL21A476MQYNNNE": "C16780", "CL05C180JB5NNNC": "C307443",
    "RC0402FR-0710KL": "C60490", "RC0402FR-0749R9L": "C87044", "RC0402FR-071KL": "C106235",
    "RC0402JR-070RL": "C60485",
}


def _res_part(value: str, size: str = "0402") -> PartType:
    key = f"R_{value}_{size}"
    if key in PARTS:
        return PARTS[key]
    code = _RES_CODE[value]
    mpn = f"RC{size}JR-070RL" if code is None else f"RC{size}FR-07{code}L"
    return _add(PartType(
        key, value if value != "0" else "0R", mpn, "YAGEO",
        f"Resistor_SMD:R_{size}_{_METRIC[size]}Metric", size,
        "Thick-film resistor, " + ("jumper" if code is None else "1 %, 1/16 W"),
        _pins("1", "2"), 0.40 if size == "0402" else 0.55, kind="res", lcsc=_LCSC.get(mpn, "")))


def _cap_part(value: str, size: str = "0402") -> PartType:
    key = f"C_{value}_{size}"
    if key in PARTS:
        return PARTS[key]
    mpn, rating, h = _CAP_MPN[(value, size)]
    return _add(PartType(
        key, value, mpn, "Samsung Electro-Mechanics",
        f"Capacitor_SMD:C_{size}_{_METRIC[size]}Metric", size,
        f"MLCC {value} {rating}", _pins("1", "2"), h, kind="cap", lcsc=_LCSC.get(mpn, "")))


# ===========================================================================
# 3. GPIO / pad assignment (TASK 1 table is generated from this)
# ===========================================================================


@dataclass(frozen=True)
class GpioUse:
    gpio: int
    net: str
    function: str
    block: str
    mux: str          # "IO_MUX" (fixed pad) | "GPIO matrix" | "GPIO" | "Strap"
    notes: str = ""


GPIO_MAP: tuple[GpioUse, ...] = (
    # --- Ethernet RMII (IO_MUX set #1 -- the ESP-IDF default for ESP32-P4) ---
    GpioUse(28, "RMII_CRS_DV", "EMAC RMII_CRS_DV", "Ethernet", "IO_MUX"),
    GpioUse(29, "RMII_RXD0", "EMAC RMII_RXD0", "Ethernet", "IO_MUX"),
    GpioUse(30, "RMII_RXD1", "EMAC RMII_RXD1", "Ethernet", "IO_MUX"),
    GpioUse(31, "ETH_MDC", "EMAC SMI MDC", "Ethernet", "GPIO matrix",
            "ESP-IDF default MDC pin"),
    GpioUse(34, "RMII_TXD0", "EMAC RMII_TXD0", "Ethernet", "IO_MUX",
            "Strap: JTAG source select, only if EFUSE_JTAG_SEL_ENABLE is burned (default: ignored); "
            "PHY input, never driven at reset"),
    GpioUse(35, "RMII_TXD1", "EMAC RMII_TXD1 + BOOT strap", "Ethernet / Boot", "IO_MUX",
            "Boot strap: 10k pull-up, BOOT button + auto-program NPN to GND"),
    GpioUse(49, "RMII_TX_EN", "EMAC RMII_TX_EN", "Ethernet", "IO_MUX"),
    GpioUse(50, "RMII_REF_CLK", "EMAC RMII_CLK (50 MHz in from PHY)", "Ethernet", "IO_MUX",
            "RMII clock is input-only on P4; LAN8720A drives it (REF_CLK-out mode)"),
    GpioUse(51, "ETH_PHY_RST_N", "PHY nRST", "Ethernet", "GPIO"),
    GpioUse(52, "ETH_MDIO", "EMAC SMI MDIO", "Ethernet", "GPIO matrix",
            "1.5k pull-up; ESP-IDF default MDIO pin"),
    # --- microSD, SDMMC slot 0 (IO_MUX only, UHS-I capable) ---
    GpioUse(39, "SD_D0", "SDMMC slot0 D0", "microSD", "IO_MUX"),
    GpioUse(40, "SD_D1", "SDMMC slot0 D1", "microSD", "IO_MUX"),
    GpioUse(41, "SD_D2", "SDMMC slot0 D2", "microSD", "IO_MUX"),
    GpioUse(42, "SD_D3", "SDMMC slot0 D3", "microSD", "IO_MUX"),
    GpioUse(43, "SD_CLK", "SDMMC slot0 CLK", "microSD", "IO_MUX"),
    GpioUse(44, "SD_CMD", "SDMMC slot0 CMD", "microSD", "IO_MUX"),
    GpioUse(20, "SD_PWR_EN", "microSD VDD switch (high = off)", "microSD", "GPIO",
            "3.3 V domain so it can fully turn off the P-FET"),
    GpioUse(21, "SD_DET", "microSD card detect (low = card)", "microSD", "GPIO"),
    # --- UART0 console / download (CP2102N) ---
    GpioUse(37, "UART0_TXD", "U0TXD -> CP2102N RXD", "Debug UART", "IO_MUX",
            "Strapping pin (boot-mode 'any value')"),
    GpioUse(38, "UART0_RXD", "U0RXD <- CP2102N TXD", "Debug UART", "IO_MUX",
            "Strapping pin (boot-mode 'any value')"),
    # --- USB Serial/JTAG (native JTAG + console on J3) ---
    GpioUse(24, "USJ_DM", "USB Serial/JTAG D-", "USB-JTAG", "IO_MUX", "22R series"),
    GpioUse(25, "USJ_DP", "USB Serial/JTAG D+", "USB-JTAG", "IO_MUX", "22R series"),
    # --- Camera / display side-band ---
    GpioUse(7, "I2C_SDA", "I2C0 SDA (camera SCCB, touch, header)", "I2C", "GPIO matrix",
            "2.2k pull-up"),
    GpioUse(8, "I2C_SCL", "I2C0 SCL (camera SCCB, touch, header)", "I2C", "GPIO matrix",
            "2.2k pull-up"),
    GpioUse(12, "CAM_EN", "Camera power enable (FPC IO0)", "Camera", "GPIO"),
    GpioUse(13, "CAM_IO1", "Camera GPIO/LED (FPC IO1)", "Camera", "GPIO"),
    GpioUse(26, "LCD_BL_PWM", "Display backlight PWM (FPC IO1)", "Display", "GPIO matrix",
            "LEDC PWM"),
    GpioUse(27, "LCD_RST", "Display reset (FPC IO0)", "Display", "GPIO"),
    # --- Active cooling ---
    GpioUse(9, "FAN_PWM", "Fan PWM (LEDC 25 kHz) -> AO3400A open-drain", "Fan", "GPIO matrix",
            "Floating at reset -> 100k gate pull-down -> fan runs at 100 % (fail-safe)"),
    GpioUse(10, "FAN_TACH", "Fan tachometer (PCNT)", "Fan", "GPIO matrix",
            "10k pull-up on the fan side + 1k series"),
    GpioUse(11, "FAN_EN", "Fan 5 V high-side switch enable", "Fan", "GPIO",
            "10k pull-up -> fan powered by default"),
    # --- UI ---
    GpioUse(6, "LED_STATUS", "Status LED (active high)", "UI", "GPIO"),
    # --- Strap-only ---
    GpioUse(36, "STRAP_GPIO36", "Download-mode qualifier strap", "Boot", "Strap",
            "10k pull-up, otherwise unused (must read 1 for joint-download boot)"),
    # --- Expansion header (J6) ---
    GpioUse(0, "GPIO0", "Header (LP GPIO / XTAL_32K_N)", "Header", "GPIO"),
    GpioUse(1, "GPIO1", "Header (LP GPIO / XTAL_32K_P)", "Header", "GPIO"),
    GpioUse(2, "GPIO2", "Header (LP GPIO / pad-JTAG MTCK)", "Header", "GPIO",
            "Weak pull-up after reset"),
    GpioUse(3, "GPIO3", "Header (LP GPIO / pad-JTAG MTDI)", "Header", "GPIO"),
    GpioUse(4, "GPIO4", "Header (LP GPIO / pad-JTAG MTMS)", "Header", "GPIO"),
    GpioUse(5, "GPIO5", "Header (LP GPIO / pad-JTAG MTDO)", "Header", "GPIO"),
    GpioUse(14, "LP_UART_TXD", "Header - LP UART TXD (LP_U0TXD)", "Header", "IO_MUX"),
    GpioUse(15, "LP_UART_RXD", "Header - LP UART RXD (LP_U0RXD)", "Header", "IO_MUX"),
    GpioUse(16, "GPIO16", "Header J6 / SPI LCD touch CS (J9)", "Header", "GPIO"),
    GpioUse(17, "GPIO17", "Header J6 / SPI LCD touch IRQ (J9)", "Header", "GPIO"),
    GpioUse(18, "GPIO18", "Header J6 / SPI LCD backlight switch (J9, high = off)", "Header", "GPIO"),
    GpioUse(19, "GPIO19", "Header J6 / SPI LCD TE input (J9)", "Header", "GPIO"),
    GpioUse(22, "GPIO22", "Header J6 / SPI LCD MISO (J9)", "Header", "GPIO"),
    GpioUse(23, "GPIO23", "Header J6 / SPI LCD CS (J9)", "Header", "GPIO"),
    GpioUse(32, "GPIO32", "Header J6 / SPI LCD SCK (J9, 22R series)", "Header", "GPIO matrix"),
    GpioUse(33, "GPIO33", "Header J6 / SPI LCD MOSI (J9)", "Header", "GPIO matrix"),
    GpioUse(53, "GPIO53", "Header J6 / SPI LCD DC (J9)", "Header", "GPIO"),
    GpioUse(54, "GPIO54", "Header J6 / SPI LCD RESET (J9)", "Header", "GPIO"),
)

# GPIO45..48 sit in the VDD_IO_5 (= VDDO_4, 0 V until firmware enables LDO4,
# then 1.8/3.3 V for UHS-I) domain -> left unconnected (reserved for 8-bit eMMC).
GPIO_NC: dict[int, str] = {g: "VDD_IO_5/VDDO_4 domain (0 V at boot, 1.8 V in UHS-I); reserved"
                           for g in (45, 46, 47, 48)}

# Dedicated (non-GPIO) SoC pads -> net.
DEDICATED_NETS: dict[str, str] = {
    # QSPI flash (dedicated MSPI pads, 0R series to U2)
    "27": "FLASH_CS", "28": "FLASH_Q", "29": "FLASH_WP", "31": "FLASH_HOLD",
    "32": "FLASH_CK", "33": "FLASH_D",
    # MIPI DSI (2 lanes)
    "34": "DSI_REXT", "35": "DSI_D1_P", "36": "DSI_D1_N", "37": "DSI_CLK_N",
    "38": "DSI_CLK_P", "39": "DSI_D0_P", "40": "DSI_D0_N",
    # MIPI CSI (2 lanes)
    "42": "CSI_D0_N", "43": "CSI_D0_P", "44": "CSI_CLK_P", "45": "CSI_CLK_N",
    "46": "CSI_D1_N", "47": "CSI_D1_P", "48": "CSI_REXT",
    # USB 2.0 HS OTG
    "49": "USBHS_DM", "50": "USBHS_DP",
    # clock / reset / DCDC control
    "99": "XTAL_N", "100": "XTAL_P", "103": "CHIP_PU",
    "78": "VDD_HP_FB", "79": "VDD_HP_EN",
    # power pads
    "9": "+3V3", "21": "+3V3", "62": "+3V3", "75": "+3V3", "77": "+3V3",
    "96": "+3V3", "101": "+3V3",
    "26": "VDD_HP", "76": "VDD_HP", "91": "VDD_HP", "54": "VDD_HP1_PAD",
    "30": "VDDO_FLASH", "71": "VDDO_FLASH",
    "59": "VDDO_PSRAM", "67": "VDDO_PSRAM", "72": "VDDO_PSRAM",
    "41": "VDD_MIPI_PHY", "73": "VDD_MIPI_PHY",
    "74": "VDDO_4_SDIO", "85": "VDDO_4_SDIO",
    "51": "VDD_USBPHY", "102": "VDD_BAT",
    "105": "GND",
}


def soc_pad_nets() -> dict[str, str]:
    """Pad -> net for U1, built from GPIO_MAP + DEDICATED_NETS."""
    nets = dict(DEDICATED_NETS)
    for use in GPIO_MAP:
        pad = p4.GPIO_PAD[use.gpio]
        assert pad not in nets, f"pad {pad} assigned twice"
        nets[pad] = use.net
    return nets


# ===========================================================================
# 4. Mechanical constants (TASK 2)
# ===========================================================================

BOARD_OUTLINE = (-55.0, -40.0, 55.0, 40.0)      # x0, y0, x1, y1 (110 x 80 mm)
BOARD_CORNER_R = 3.0
KEEPOUT_HALF = 12.5                             # 25 x 25 mm heatsink keep-out
KEEPOUT_MAX_HEIGHT_MM = 0.60                    # 0402 passives are <= 0.55 mm
HEATSINK_HOLES = {"H1": (-15.0, -15.0), "H2": (15.0, -15.0),
                  "H3": (-15.0, 15.0), "H4": (15.0, 15.0)}
HEATSINK_HOLE_KEEPOUT_R = 3.5                   # standoff/washer + clearance
BOARD_HOLES = {"H5": (-51.0, -36.0), "H6": (51.0, -36.0),
               "H7": (-51.0, 36.0), "H8": (51.0, 36.0)}
FIDUCIALS = {"FID1": (-52.0, 32.0), "FID2": (52.0, 28.0), "FID3": (47.0, -24.0)}

THERMAL_VIA = {"drill_mm": 0.30, "pad_mm": 0.60, "pitch_mm": 1.0, "grid": 7,
               "fill": "IPC-4761 Type VII (resin-filled + copper-capped, VIPPO)",
               "note": "The generated U1 footprint uses ONE solid 7.5 mm EPAD copper pad (paste split "
                       "into 3x3 windows), so vias may sit in the paste gaps or, being VIPPO, under paste."}
BOTTOM_SPREADER_MM = 14.0                       # exposed B.Cu GND heat spreader under U1 (square)

HEATSINK = {
    "base_mm": (25.0, 25.0), "height_mm": 10.0, "material": "Al 6063-T5, black anodised",
    "tim": "0.1 mm phase-change pad (e.g. Laird Tpcm 580) -- flush on SoC lid",
    "clamp": "4 x M2.5 spring-loaded screws through H1-H4 into a 30 x 30 mm X-bracket",
    "fan": "25 x 25 x 10 mm or 30 x 30 x 7 mm, 5 V, 4-wire PWM, screwed to fin top",
}

# ===========================================================================
# 5. Netlist construction
# ===========================================================================

COMPONENTS: list[Component] = []
_counters: Counter[str] = Counter()


def _next_ref(prefix: str) -> str:
    _counters[prefix] += 1
    return f"{prefix}{_counters[prefix]}"


def add(ref: str, part: PartType, conns: dict[str, str], group: str, *,
        place: Place | Near | None = None, value: str = "", dnp: bool = False,
        note: str = "") -> Component:
    """Instantiate a part. ``conns`` keys may be pad numbers or pin names."""
    resolved = {part.pad(k): v for k, v in conns.items()}
    comp = Component(ref, part, resolved, group, value, dnp, place, note)
    COMPONENTS.append(comp)
    return comp


def R(value: str, a: str, b: str, group: str, *, size: str = "0402",
      place: Place | Near | None = None, dnp: bool = False, note: str = "") -> Component:
    return add(_next_ref("R"), _res_part(value, size), {"1": a, "2": b}, group,
               place=place, dnp=dnp, note=note)


def C(value: str, a: str, b: str, group: str, *, size: str = "0402",
      place: Place | Near | None = None, dnp: bool = False, note: str = "") -> Component:
    return add(_next_ref("C"), _cap_part(value, size), {"1": a, "2": b}, group,
               place=place, dnp=dnp, note=note)


GND = "GND"

# ---------------------------------------------------------------- U1 SoC ---
U1 = add("U1", ESP32P4, soc_pad_nets(), "SoC", place=Place(0.0, 0.0, 0.0))

# Decoupling: (pad, net, [values]) -- every cap within 2 mm of its pad.
SOC_DECOUPLING: tuple[tuple[str, str, tuple[str, ...]], ...] = (
    ("9", "+3V3", ("100nF", "10uF")),            # VDD_LP
    ("21", "+3V3", ("100nF",)),                  # VDD_IO_0
    ("26", "VDD_HP", ("100nF", "1uF", "10uF")),  # VDD_HP_0 (rail entry)
    ("30", "VDDO_FLASH", ("100nF", "1uF")),      # VDD_FLASH_IO
    ("41", "VDD_MIPI_PHY", ("10nF", "100nF", "1uF")),
    ("51", "VDD_USBPHY", ("10nF", "100nF", "4.7uF")),
    ("54", "VDD_HP1_PAD", ("100nF", "1uF")),     # VDD_HP_1
    ("59", "VDDO_PSRAM", ("100nF", "1uF")),      # VDD_PSRAM_0
    ("62", "+3V3", ("100nF", "1uF")),            # VDD_IO_4
    ("67", "VDDO_PSRAM", ("100nF", "1uF")),      # VDD_PSRAM_1
    ("71", "VDDO_FLASH", ("1uF",)),              # LDO VO1 output
    ("72", "VDDO_PSRAM", ("1uF",)),              # LDO VO2 output
    ("73", "VDD_MIPI_PHY", ("1uF",)),            # LDO VO3 output
    ("74", "VDDO_4_SDIO", ("1uF",)),             # LDO VO4 output
    ("75", "+3V3", ("100nF", "10uF")),           # VDD_LDO
    ("76", "VDD_HP", ("100nF", "1uF", "10uF")),  # VDD_HP_2
    ("77", "+3V3", ("100nF", "10uF")),           # VDD_DCDCC
    ("85", "VDDO_4_SDIO", ("100nF",)),           # VDD_IO_5
    ("91", "VDD_HP", ("100nF", "1uF")),          # VDD_HP_3
    ("96", "+3V3", ("100nF",)),                  # VDD_IO_6
    ("101", "+3V3", ("10nF", "100nF")),          # VDD_ANA
    ("102", "VDD_BAT", ("100nF", "10uF")),       # VDD_BAT
)
for _pad, _net, _vals in SOC_DECOUPLING:
    for _v in _vals:
        C(_v, _net, GND, "SoC decoupling", place=Near("U1", _pad, 2.0),
          note=f"U1 pad {_pad} {p4.PAD_NAME[_pad]}")

_G = "SoC support"
R("0", "+3V3", "VDD_USBPHY", _G, place=Near("U1", "51", 2.0), note="VDD_USBPHY feed (Espressif R40)")
R("0", "+3V3", "VDD_BAT", _G, place=Near("U1", "102", 2.0), note="VDD_BAT from 3V3 (no coin cell)")
R("0", "VDD_HP", "VDD_HP1_PAD", _G, place=Near("U1", "54", 2.0), note="Espressif R39 (pad 54 rev-compat)")
R("4.02k", "DSI_REXT", GND, _G, place=Near("U1", "34", 2.0), note="MIPI DSI bias, 1 %")
R("4.02k", "CSI_REXT", GND, _G, place=Near("U1", "48", 2.0), note="MIPI CSI bias, 1 %")
R("10k", "+3V3", "CHIP_PU", _G, place=Near("U1", "103", 2.0), note="CHIP_PU RC delay (10k/1uF)")
C("1uF", "CHIP_PU", GND, _G, place=Near("U1", "103", 2.0), note="CHIP_PU RC delay")
R("10k", "+3V3", "RMII_TXD1", _G, place=Near("U1", "66", 3.0), note="GPIO35 boot strap pull-up (SPI boot)")
R("10k", "+3V3", "STRAP_GPIO36", _G, place=Near("U1", "68", 3.0), note="GPIO36 strap pull-up")
R("1k", "RMII_TXD1", "BOOT_BTN", _G, place=Near("U1", "66", 3.0),
  note="Isolates the ~50 mm BOOT button / auto-program stub from the 50 MHz RMII TXD1 line; "
       "pressed: 3.3 V x 1k/11k = 0.3 V < VIL")
R("1M", "USBHS_DP", GND, _G, place=Near("U1", "50", 2.0), dnp=True,
  note="Espressif rev1.x DP pull-down, DNP on rev v3.x")

# 40 MHz crystal: 0R series at the chip, load caps at the crystal (>= 4.5 mm away).
R("0", "XTAL_P", "XTAL_P_Y", "Clock", place=Near("U1", "100", 2.0), note="Espressif R3")
R("0", "XTAL_N", "XTAL_N_Y", "Clock", place=Near("U1", "99", 2.0), note="Espressif R4")
add("Y1", XTAL40, {"XIN": "XTAL_P_Y", "XOUT": "XTAL_N_Y", "2": GND, "4": GND}, "Clock",
    place=Place(-3.5, -15.0, 0.0),
    note="Outside keep-out; Espressif requires >= 4.5 mm from the clock pads, no vias on XTAL traces")
C("12pF", "XTAL_P_Y", GND, "Clock", place=Near("Y1", "1", 2.0),
  note="C_L 10 pF, ~4 pF stray incl. the 10 mm trace; tune on first article")
C("12pF", "XTAL_N_Y", GND, "Clock", place=Near("Y1", "3", 2.0),
  note="C_L 10 pF, ~4 pF stray incl. the 10 mm trace; tune on first article")

# ------------------------------------------------------------- U2 Flash ---
_G = "QSPI flash"
for _sig in ("CS", "Q", "WP", "HOLD", "CK", "D"):
    R("0", f"FLASH_{_sig}", f"FLASH_{_sig}_M", _G, place=Near("U1", {
        "CS": "27", "Q": "28", "WP": "29", "HOLD": "31", "CK": "32", "D": "33"}[_sig], 2.5),
      note="Espressif-recommended series footprint (drive/EMI tuning)")
add("U2", FLASH, {"~CS": "FLASH_CS_M", "DO/IO1": "FLASH_Q_M", "~WP/IO2": "FLASH_WP_M",
                  "GND": GND, "DI/IO0": "FLASH_D_M", "CLK": "FLASH_CK_M",
                  "~HOLD/IO3": "FLASH_HOLD_M", "VCC": "VDDO_FLASH", "EP": GND}, _G,
    place=Place(-20.0, 5.0, 90.0))
R("10k", "VDDO_FLASH", "FLASH_CS_M", _G, place=Near("U2", "1"), note="CS pull-up (Espressif)")
C("100nF", "VDDO_FLASH", GND, _G, place=Near("U2", "8", 2.0))

# ------------------------------------------------ U4 VDD_HP external DCDC ---
_G = "VDD_HP DCDC"
add("U4", BUCK_HP, {"EN": "VDD_HP_EN", "GND": GND, "SW": "HP_SW", "VIN": "+3V3",
                    "FB": "VDD_HP_FB"}, _G, place=Place(16.5, -7.5, 0.0),
    note="Espressif-verified model; EN/FB fully controlled by ESP32-P4")
add("L2", IND_HP, {"1": "HP_SW", "2": "VDD_HP"}, _G, place=Place(20.0, -8.5, 0.0))
C("4.7uF", "+3V3", GND, _G, place=Near("U4", "4", 2.0), note="Espressif C1")
C("22uF", "VDD_HP", GND, _G, size="0805", place=Near("L2", "2"), note="Espressif C3")
R("499k", "VDD_HP", "VDD_HP_FB", _G, place=Near("U4", "5"), note="Espressif R1 (rev v3.x: populate)")
R("499k", "VDD_HP_FB", GND, _G, place=Near("U4", "5"), note="Espressif R2 (rev v3.x: populate)")
C("22pF", "VDD_HP", "VDD_HP_FB", _G, place=Near("U4", "5"), note="Espressif C2 feed-forward")

# ------------------------------------------------------- Power input path ---
_G = "Power input"
for _i, (_vb, _pos) in enumerate((("VBUS1", (22.0, 28.5)), ("VBUS2", (-42.0, 25.0)),
                                  ("VBUS3", (-42.0, 11.0))), start=1):
    add(f"D{_i}", SCHOTTKY, {"A": _vb, "K": "VSYS_OR"}, _G, place=Place(*_pos, 0.0),
        note="VBUS OR-ing (no back-feed); ~1 W at 2.5 A -> >= 100 mm^2 copper on both pads")
add("F1", PTC, {"1": "VSYS_OR", "2": "VSYS_5V"}, _G, place=Place(-34.0, 27.0, 0.0))
C("10uF", "VSYS_5V", GND, _G, size="0805", place=Near("F1", "2"))

# -------------------------------------------------- U3 5 V -> 3.3 V buck ---
_G = "3V3 buck"
add("U3", BUCK_3V3, {"PG": "BUCK_PG", "FB": "BUCK_FB", "AGND": GND,
                     "FSW": GND, "DEF": GND, "SS/TR": "BUCK_SS", "AVIN": "VSYS_5V",
                     "11": "VSYS_5V", "12": "VSYS_5V", "EN": "VSYS_5V", "VOS": "+3V3",
                     "15": GND, "16": GND, "EP": GND, "1": "BUCK_SW", "2": "BUCK_SW",
                     "3": "BUCK_SW"}, _G,
    place=Place(-34.0, 18.0, 0.0),
    note="FSW=GND -> 2.5 MHz (FSW must be low at start-up), DEF=GND -> nominal Vout")
add("L1", IND_3V3, {"1": "BUCK_SW", "2": "+3V3"}, _G, place=Place(-27.0, 20.0, 0.0))
C("10uF", "VSYS_5V", GND, _G, size="0805", place=Near("U3", "11"))
C("10uF", "VSYS_5V", GND, _G, size="0805", place=Near("U3", "12"))
C("100nF", "VSYS_5V", GND, _G, place=Near("U3", "10", 2.0))
C("22uF", "+3V3", GND, _G, size="0805", place=Near("L1", "2"))
C("22uF", "+3V3", GND, _G, size="0805", place=Near("L1", "2"))
C("47uF", "+3V3", GND, _G, size="0805", place=Place(-14.5, -8.0, 90.0),
  note="Bulk at the SoC power entry, just outside the keep-out, clear of H1")
C("47uF", "+3V3", GND, _G, size="0805", place=Place(14.5, 8.0, 90.0),
  note="Bulk at the SoC power entry, just outside the keep-out, clear of H4")
C("3.3nF", "BUCK_SS", GND, _G, place=Near("U3", "9"), note="Soft-start ~1 ms")
R("75k", "+3V3", "BUCK_FB", _G, place=Near("U3", "5"), note="Vout = 0.8 V x (1 + 75k/24k) = 3.30 V")
R("24k", "BUCK_FB", GND, _G, place=Near("U3", "5"))
R("100k", "+3V3", "BUCK_PG", _G, place=Near("U3", "4"))
add("D4", LED_G, {"A": "LED_PWR_A", "K": GND}, "UI", place=Place(-43.0, -30.0, 0.0),
    note="3V3 rail indicator")
R("1k", "+3V3", "LED_PWR_A", "UI", place=Near("D4", "2"))

# ------------------------------------------------------------ USB ports ---


def usb_port(ref: str, vbus: str, dp: str, dm: str, esd: PartType, esd_ref: str,
             group: str, place: Place) -> None:
    """USB-C UFP: 5.1k Rd on each CC, ESD at the connector, 10 uF on VBUS."""
    add(ref, USB_C, {"A4": vbus, "A9": vbus, "B4": vbus, "B9": vbus,
                     "A6": dp, "B6": dp, "A7": dm, "B7": dm,
                     "A5": f"{ref}_CC1", "B5": f"{ref}_CC2",
                     "A1": GND, "A12": GND, "B1": GND, "B12": GND, "SH": GND}, group,
        place=place)
    R("5.1k", f"{ref}_CC1", GND, group, place=Near(ref, "A5"), note="UFP Rd")
    R("5.1k", f"{ref}_CC2", GND, group, place=Near(ref, "B5"), note="UFP Rd")
    if esd is ESD_USB:           # USBLC6-2SC6: flow-through, VBUS clamp
        esd_conns = {"1": dp, "6": dp, "3": dm, "4": dm, "VBUS": vbus, "GND": GND}
    else:                        # TPD2EUSB30: D+, D-, GND
        esd_conns = {"D+": dp, "D-": dm, "GND": GND}
    add(esd_ref, esd, esd_conns, group, place=Near(ref, "A6", 5.0))
    C("10uF", vbus, GND, group, size="0805", place=Near(ref, "A4"))


# J1: native USB 2.0 High-Speed OTG (UTMI PHY)
_G = "USB1 HS"
usb_port("J1", "VBUS1", "USB1_DP", "USB1_DM", ESD_USB_HS, "U7", _G,
         Place(22.0, 36.0, 0.0, faces="bottom"))
R("0", "USB1_DP", "USBHS_DP", _G, place=Near("U1", "50", 2.5), note="Espressif: reserve R/C near SoC")
R("0", "USB1_DM", "USBHS_DM", _G, place=Near("U1", "49", 2.5), note="Espressif: reserve R/C near SoC")

# J2: debug UART through CP2102N (self-powered from +3V3)
_G = "USB2 debug UART"
usb_port("J2", "VBUS2", "USB2_DP", "USB2_DM", ESD_USB, "U8", _G,
         Place(-51.0, 22.0, 0.0, faces="left"))
add("U6", UART_BRIDGE, {"D+": "USB2_DP", "D-": "USB2_DM", "VIO": "+3V3", "VDD": "+3V3",
                        "VREGIN": "+3V3", "VBUS": "CP_VBUS_SENSE", "~RST": "CP_RST_N",
                        "~RTS": "CP_RTS", "~DTR": "CP_DTR", "RXD": "UART0_TXD",
                        "TXD": "UART0_RXD", "2": GND, "25": GND}, _G,
    place=Place(-42.0, 19.0, 0.0), note="Self-powered configuration")
R("22.1k", "VBUS2", "CP_VBUS_SENSE", _G, place=Near("U6", "8"), note="VBUS sense divider")
R("47.5k", "CP_VBUS_SENSE", GND, _G, place=Near("U6", "8"), note="VBUS sense divider")
R("1k", "+3V3", "CP_RST_N", _G, place=Near("U6", "9"))
C("100nF", "+3V3", GND, _G, place=Near("U6", "6", 2.0))
C("1uF", "+3V3", GND, _G, place=Near("U6", "6", 2.0))
C("100nF", "+3V3", GND, _G, place=Near("U6", "7", 2.0))
C("1uF", "+3V3", GND, _G, place=Near("U6", "7", 2.0))
C("100nF", "+3V3", GND, _G, place=Near("U6", "5", 2.0))
# Classic Espressif auto-program circuit (esptool DTR/RTS sequence).
add("Q1", DUAL_NPN, {"B1": "Q1_B1", "E1": "CP_RTS", "C1": "CHIP_PU",
                     "B2": "Q1_B2", "E2": "CP_DTR", "C2": "BOOT_BTN"}, _G,
    place=Near("U6", "19", 6.0), note="Q1A drives CHIP_PU, Q1B drives GPIO35 (BOOT)")
R("10k", "CP_DTR", "Q1_B1", _G, place=Near("Q1", "2"))
R("10k", "CP_RTS", "Q1_B2", _G, place=Near("Q1", "5"))

# J3: native USB Serial/JTAG (GPIO24/25) -> JTAG + console with zero config
_G = "USB3 USB-Serial-JTAG"
usb_port("J3", "VBUS3", "USB3_DP", "USB3_DM", ESD_USB, "U9", _G,
         Place(-51.0, 8.0, 0.0, faces="left"))
R("22", "USB3_DP", "USJ_DP", _G, place=Near("U1", "53", 2.5), note="Espressif FS series R")
R("22", "USB3_DM", "USJ_DM", _G, place=Near("U1", "52", 2.5), note="Espressif FS series R")

# ------------------------------------------------------------ Ethernet ---
_G = "Ethernet"
add("U5", PHY, {"VDD2A": "PHY_AVDD", "LED2/~INTSEL": "PHY_LED2", "LED1/REGOFF": "PHY_LED1",
                "XTAL2": "PHY_XO", "XTAL1/CLKIN": "PHY_XI", "VDDCR": "PHY_VDDCR",
                "RXD1/MODE1": "RMII_RXD1_PHY", "RXD0/MODE0": "RMII_RXD0_PHY",
                "VDDIO": "+3V3", "RXER/PHYAD0": "PHY_RXER", "CRS_DV/MODE2": "RMII_CRS_DV_PHY",
                "MDIO": "ETH_MDIO", "MDC": "ETH_MDC", "~INT/REFCLKO": "RMII_REF_CLK_PHY",
                "~RST": "ETH_PHY_RST_N", "TXEN": "RMII_TX_EN_PHY", "TXD0": "RMII_TXD0_PHY",
                "TXD1": "RMII_TXD1_PHY", "VDD1A": "PHY_AVDD", "TXN": "ETH_TX_N",
                "TXP": "ETH_TX_P", "RXN": "ETH_RX_N", "RXP": "ETH_RX_P",
                "RBIAS": "PHY_RBIAS", "VSS": GND}, _G, place=Place(22.0, 3.0, 0.0))
# RMII series terminations: at the driver (SoC for TX, PHY for RX/CLK).
for _net, _anchor in (("RMII_TXD0", ("U1", "65")), ("RMII_TXD1", ("U1", "66")),
                      ("RMII_TX_EN", ("U1", "92"))):
    R("22", _net, f"{_net}_PHY", _G, place=Near(*_anchor, 2.5), note="Source termination at SoC")
for _net, _pad in (("RMII_RXD0", "8"), ("RMII_RXD1", "7"), ("RMII_CRS_DV", "11"),
                   ("RMII_REF_CLK", "14")):
    R("33" if _net == "RMII_REF_CLK" else "22", f"{_net}_PHY", _net, _G,
      place=Near("U5", _pad), note="Source termination at PHY")
add("Y2", XTAL25, {"XIN": "PHY_XI", "XOUT": "PHY_XO", "2": GND, "4": GND}, _G,
    place=Place(22.0, 9.5, 0.0))
C("18pF", "PHY_XI", GND, _G, place=Near("Y2", "1"), note="C_L = 12 pF: 2 x (12 - 3 pF stray)")
C("18pF", "PHY_XO", GND, _G, place=Near("Y2", "3"), note="C_L = 12 pF: 2 x (12 - 3 pF stray)")
R("12.1k", "PHY_RBIAS", GND, _G, place=Near("U5", "24", 2.0), note="RBIAS 1 %")
R("10k", "PHY_LED2", GND, _G, place=Near("U5", "2"), note="nINTSEL = 0 -> REF_CLK-out (50 MHz on pin 14)")
R("10k", "PHY_RXER", GND, _G, place=Near("U5", "10"), note="PHYAD0 = 0")
R("10k", "+3V3", "ETH_PHY_RST_N", _G, place=Near("U5", "15"))
R("1.5k", "+3V3", "ETH_MDIO", _G, place=Near("U5", "12"), note="MDIO pull-up")
add("FB1", FERRITE, {"1": "+3V3", "2": "PHY_AVDD"}, _G, place=Near("U5", "1", 4.0))
for _v in ("1uF", "100nF"):
    C(_v, "PHY_AVDD", GND, _G, place=Near("U5", "1", 2.0))
    C(_v, "PHY_AVDD", GND, _G, place=Near("U5", "19", 2.0))
C("100nF", "+3V3", GND, _G, place=Near("U5", "9", 2.0))
C("1uF", "PHY_VDDCR", GND, _G, place=Near("U5", "6", 2.0))
C("470pF", "PHY_VDDCR", GND, _G, place=Near("U5", "6", 2.0))
for _net in ("ETH_TX_P", "ETH_TX_N", "ETH_RX_P", "ETH_RX_N"):
    R("49.9", _net, "PHY_AVDD", _G, place=Near("U5", {"ETH_TX_P": "21", "ETH_TX_N": "20",
                                                      "ETH_RX_P": "23", "ETH_RX_N": "22"}[_net]),
      note="LAN8720A line termination")
add("J4", RJ45, {"TD+": "ETH_TX_P", "TD-": "ETH_TX_N", "RD+": "ETH_RX_P", "RD-": "ETH_RX_N",
                 "TCT": "ETH_CT", "RCT": "ETH_CT", "GND": GND,
                 "LED_G_A": "ETH_LED_LINK_A", "LED_G_K": GND,
                 "LED_Y_A": "ETH_LED_SPEED_A", "LED_Y_K": GND, "SH": GND}, _G,
    place=Place(46.0, 3.0, 0.0, faces="right"))
add("FB2", FERRITE, {"1": "PHY_AVDD", "2": "ETH_CT"}, _G, place=Near("J4", "4", 6.0))
C("100nF", "ETH_CT", GND, _G, place=Near("J4", "4", 6.0))
C("10nF", "ETH_CT", GND, _G, place=Near("J4", "4", 6.0))
R("330", "PHY_LED1", "ETH_LED_LINK_A", _G, place=Near("U5", "3"), note="Link/activity LED")
R("330", "PHY_LED2", "ETH_LED_SPEED_A", _G, place=Near("U5", "2"), note="Speed LED")

# ------------------------------------------------------------- microSD ---
_G = "microSD"
_SD_SIG = {"D0": "80", "D1": "81", "D2": "82", "D3": "83", "CLK": "84", "CMD": "86"}
for _sig, _pad in _SD_SIG.items():
    R("0", f"SD_{_sig}", f"SD_{_sig}_C", _G, place=Near("U1", _pad, 2.5),
      note="Espressif: reserve series R on every SDIO line")
C("10pF", "SD_CLK_C", GND, _G, place=Near("U1", "84", 3.0), dnp=True,
  note="Espressif: reserve CLK cap for tuning")
for _sig in ("D0", "D1", "D2", "D3", "CMD"):
    R("51k", f"SD_{_sig}_C", "VDDO_4_SDIO", _G, place=Near("J5", None, 6.0),
      note="Pull-up to VDDO_4 (1.8/3.3 V UHS-I switching)")
add("J5", MICROSD, {"DAT2": "SD_D2_C", "DAT3/CD": "SD_D3_C", "CMD": "SD_CMD_C",
                    "VDD": "SD_VDD", "CLK": "SD_CLK_C", "VSS": GND, "DAT0": "SD_D0_C",
                    "DAT1": "SD_D1_C", "DET_A": "SD_DET", "DET_B": GND, "SHIELD": GND}, _G,
    place=Place(6.0, -31.0, 0.0, faces="top"))
add("Q2", PMOS, {"G": "SD_PWR_EN", "S": "+3V3", "D": "SD_VDD"}, _G, place=Near("J5", "4", 6.0),
    note="SD power switch, gate low = on")
R("10k", "SD_PWR_EN", GND, _G, place=Near("Q2", "1"), note="Card powered by default")
R("10k", "+3V3", "SD_DET", _G, place=Near("J5", "10"))
C("10uF", "SD_VDD", GND, _G, size="0805", place=Near("J5", "4"))
# Flow-through: pads 10/9/7/6 sit opposite 1/2/4/5 and carry the same net so the line
# is routed straight across the device.
add("U10", ESD_4CH, {"D1+": "SD_D0_C", "10": "SD_D0_C", "D1-": "SD_D1_C", "9": "SD_D1_C",
                     "D2+": "SD_D2_C", "7": "SD_D2_C", "D2-": "SD_D3_C", "6": "SD_D3_C",
                     "3": GND, "8": GND}, _G, place=Near("J5", "7", 6.0),
    note="Flow-through TVS, 0.5 pF (UHS-I safe)")
add("U11", ESD_4CH, {"D1+": "SD_CLK_C", "10": "SD_CLK_C", "D1-": "SD_CMD_C", "9": "SD_CMD_C",
                     "D2+": "SD_DET", "7": "SD_DET", "3": GND, "8": GND}, _G,
    place=Near("J5", "5", 6.0), note="Flow-through TVS; channel D2- (pads 5/6) unused")
C("100nF", "SD_VDD", GND, _G, place=Near("J5", "4", 2.0))

# -------------------------------------------------- Camera (MIPI CSI-2) ---
_G = "Camera CSI"
for _lane, _pads in (("D0", ("43", "42")), ("D1", ("47", "46")), ("CLK", ("44", "45"))):
    for _pol, _pad in zip(("P", "N"), _pads):
        R("0", f"CSI_{_lane}_{_pol}", f"CAM_{_lane}_{_pol}", _G, place=Near("U1", _pad, 2.5),
          note="Espressif: reserve 0R on MIPI lines")
add("J_CAM", FPC22, {"D0_N": "CAM_D0_N", "D0_P": "CAM_D0_P", "D1_N": "CAM_D1_N",
                     "D1_P": "CAM_D1_P", "CLK_N": "CAM_CLK_N", "CLK_P": "CAM_CLK_P",
                     "IO0": "CAM_EN", "IO1": "CAM_IO1", "SCL": "I2C_SCL", "SDA": "I2C_SDA",
                     "3V3": "+3V3", "MP": GND,
                     **{p: GND for p, n in RPI22_PINS if n == "GND"}}, _G,
    place=Place(6.0, 35.0, 0.0, faces="bottom"),
    note="2-lane CSI; D2/D3 not connected")
C("1uF", "+3V3", GND, _G, place=Near("J_CAM", "22"))
C("100nF", "+3V3", GND, _G, place=Near("J_CAM", "22"))

# ------------------------------------------------- Display (MIPI DSI) ---
_G = "Display DSI"
for _lane, _pads in (("D0", ("39", "40")), ("D1", ("35", "36")), ("CLK", ("38", "37"))):
    for _pol, _pad in zip(("P", "N"), _pads):
        R("0", f"DSI_{_lane}_{_pol}", f"DISP_{_lane}_{_pol}", _G, place=Near("U1", _pad, 2.5),
          note="Espressif: reserve 0R on MIPI lines")
add("J_DSI", FPC22, {"D0_N": "DISP_D0_N", "D0_P": "DISP_D0_P", "D1_N": "DISP_D1_N",
                     "D1_P": "DISP_D1_P", "CLK_N": "DISP_CLK_N", "CLK_P": "DISP_CLK_P",
                     "IO0": "LCD_RST", "IO1": "LCD_BL_PWM", "SCL": "I2C_SCL",
                     "SDA": "I2C_SDA", "3V3": "+3V3", "MP": GND,
                     **{p: GND for p, n in RPI22_PINS if n == "GND"}}, _G,
    place=Place(-14.0, 35.0, 0.0, faces="bottom"),
    note="2-lane DSI; D2/D3 not connected")
C("1uF", "+3V3", GND, _G, place=Near("J_DSI", "22"))
C("100nF", "+3V3", GND, _G, place=Near("J_DSI", "22"))
add("J8", JST_SH2, {"1": "VSYS_5V", "2": GND, "MP": GND}, _G,
    place=Place(-28.0, 36.5, 0.0, faces="bottom"), note="5 V backlight feed for DSI panels")

# ----------------------------------------------------------------- I2C ---
R("2.2k", "+3V3", "I2C_SDA", "I2C", place=Near("J_CAM", "21", 8.0))
R("2.2k", "+3V3", "I2C_SCL", "I2C", place=Near("J_CAM", "20", 8.0))

# --------------------------------------------------------- Fan control ---
_G = "Fan control"
add("J7", FAN_HDR, {"GND": GND, "+5V": "FAN_5V", "TACH": "FAN_TACH_RAW", "PWM": "FAN_PWM_OD"},
    _G, place=Place(-30.0, -30.0, 0.0), note="Intel 4-wire; keep >= 10 mm from heatsink exhaust")
add("Q3", NMOS, {"G": "FAN_PWM_G", "S": GND, "D": "FAN_PWM_OD"}, _G, place=Near("J7", "4", 6.0),
    note="Open-drain PWM driver (fan has internal pull-up); logic inverted in firmware")
R("100", "FAN_PWM", "FAN_PWM_G", _G, place=Near("Q3", "1"), note="Gate series R")
R("100k", "FAN_PWM_G", GND, _G, place=Near("Q3", "1"), note="Off at reset -> PWM high -> 100 % fan")
R("10k", "FAN_PWM_OD", "FAN_5V", _G, place=Near("Q3", "3"), dnp=True,
  note="Only for fans without internal PWM pull-up")
add("Q4", PMOS, {"G": "FAN_HS_G", "S": "VSYS_5V", "D": "FAN_5V"}, _G, place=Near("J7", "2", 6.0),
    note="High-side 5 V switch (zero-RPM / sleep)")
R("100k", "VSYS_5V", "FAN_HS_G", _G, place=Near("Q4", "1"))
add("Q5", NMOS, {"G": "FAN_EN", "S": GND, "D": "FAN_HS_G"}, _G, place=Near("Q4", "1", 5.0))
R("10k", "+3V3", "FAN_EN", _G, place=Near("Q5", "1"), note="Fan powered by default")
R("10k", "+3V3", "FAN_TACH_RAW", _G, place=Near("J7", "3", 6.0), note="Open-collector tach pull-up")
R("1k", "FAN_TACH_RAW", "FAN_TACH", _G, place=Near("J7", "3", 6.0), note="Injection-current limit")
C("10uF", "FAN_5V", GND, _G, size="0805", place=Near("J7", "2", 6.0))

# --------------------------------------------------------- Buttons / LEDs ---
_G = "UI"
add("SW1", TACT, {"1": "CHIP_PU", "2": GND}, _G,
    place=Place(-40.0, -35.0, 0.0), note="RESET")
add("SW2", TACT, {"1": "BOOT_BTN", "2": GND}, _G,
    place=Place(-33.0, -35.0, 0.0), note="BOOT (GPIO35)")
add("D5", LED_R, {"A": "LED_STATUS_A", "K": GND}, _G, place=Place(-43.0, -27.0, 0.0),
    note="User status LED")
R("1k", "LED_STATUS", "LED_STATUS_A", _G, place=Near("D5", "2"))

# ------------------------------------------------------- Expansion header ---
HEADER_PINOUT: tuple[str, ...] = (
    "+3V3", "VSYS_5V", "GPIO0", "GPIO1", "GPIO2", "GPIO3", "GPIO4", "GPIO5", GND, GND,
    "I2C_SDA", "I2C_SCL", "LP_UART_TXD", "LP_UART_RXD", "GPIO16", "GPIO17", "GPIO18",
    "GPIO19", "GPIO22", "GPIO23", "GPIO32", "GPIO33", "GPIO53", "GPIO54",
)
add("J6", HDR_2X12, {str(i): n for i, n in enumerate(HEADER_PINOUT, 1)}, "Header",
    place=Place(-50.5, -27.0, 0.0),
    note="Footprint origin = pin 1; rows run +Y to y = +0.9 mm")

# ------------------------------------------- SPI / I2C TFT display header ---
# Second display option next to the MIPI-DSI FPC: the common ST7789 / ILI9341 /
# ST7735 SPI modules (with or without XPT2046 or I2C capacitive touch) connect
# with dupont wires. Signals are shared with the expansion header J6.
_G = "Display SPI"
LCD_HEADER_PINOUT: tuple[str, ...] = (
    "+3V3", "VSYS_5V", GND, GND, "LCD_SPI_SCK", "GPIO33", "GPIO22", "GPIO23",
    "GPIO53", "GPIO54", "LCD_SPI_BL", "GPIO16", "GPIO17", "GPIO19", "I2C_SDA", "I2C_SCL",
)
LCD_HEADER_LABELS: tuple[str, ...] = (
    "3V3", "5V", "GND", "GND", "SCK", "MOSI", "MISO", "CS", "DC", "RST", "BL", "T_CS",
    "T_IRQ", "TE", "SDA", "SCL",
)
add("J9", HDR_2X8, {str(i): n for i, n in enumerate(LCD_HEADER_PINOUT, 1)}, _G,
    place=Place(24.0, -35.0, 90.0),
    note="SPI TFT header; silkscreen labels = LCD_HEADER_LABELS")
R("22", "GPIO32", "LCD_SPI_SCK", _G, place=Near("J9", "5", 6.0), note="SPI CLK series R (Espressif EMC rule)")
C("10pF", "LCD_SPI_SCK", GND, _G, place=Near("J9", "5", 6.0), dnp=True, note="Optional CLK edge filter")
add("Q6", PMOS, {"G": "GPIO18", "S": "+3V3", "D": "LCD_SPI_BL"}, _G, place=Near("J9", "11", 6.0),
    note="Backlight high-side switch: safe for modules without an on-board BL transistor")
R("100k", "GPIO18", GND, _G, place=Near("Q6", "1"), note="Backlight on at reset")
C("100nF", "LCD_SPI_BL", GND, _G, place=Near("Q6", "3"))

# ------------------------------------------------------------ Mechanical ---
for _ref, (_x, _y) in HEATSINK_HOLES.items():
    add(_ref, MH_M25, {"1": GND}, "Mechanical", place=Place(_x, _y, 0.0),
        note="Heatsink/fan clamp, plated, tied to GND")
for _ref, (_x, _y) in BOARD_HOLES.items():
    add(_ref, MH_M3, {"1": GND}, "Mechanical", place=Place(_x, _y, 0.0))


# ===========================================================================
# 6. Queries + validation
# ===========================================================================

def component(ref: str) -> Component:
    for comp in COMPONENTS:
        if comp.ref == ref:
            return comp
    raise KeyError(ref)


def nets() -> dict[str, list[tuple[str, str]]]:
    """net -> [(ref, pad), ...]"""
    out: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for comp in COMPONENTS:
        for pad, net in comp.conns.items():
            out[net].append((comp.ref, pad))
    return dict(out)


NETCLASS_PATTERNS: tuple[tuple[str, str], ...] = (
    ("USB_90", r"^(USB1_D[PM]|USBHS_D[PM]|USB2_D[PM]|USB3_D[PM]|USJ_D[PM])$"),
    ("MIPI_100", r"^(CSI|CAM|DSI|DISP)_(D[01]|CLK)_[PN]$"),
    ("ETH_100", r"^ETH_(TX|RX)_[PN]$"),
    ("SE_50", r"^(SD_(D[0-3]|CLK|CMD)(_C)?|RMII_.*|FLASH_(CS|Q|WP|HOLD|CK|D)(_M)?|XTAL_[PN](_Y)?)$"),
    ("POWER", r"^(\+3V3|VSYS_5V|VSYS_OR|VBUS[123]|VDD_HP|VDD_HP1_PAD|BUCK_SW|HP_SW|FAN_5V|SD_VDD)$"),
)


def netclass_of(net: str) -> str:
    for cls, pat in NETCLASS_PATTERNS:
        if re.match(pat, net):
            return cls
    return "Default"


DIFF_PAIRS: tuple[tuple[str, str, str], ...] = tuple(
    (f"{base}_P" if not base.startswith("USB") and not base.startswith("USJ") else base + "P",
     f"{base}_N" if not base.startswith("USB") and not base.startswith("USJ") else base + "M",
     cls)
    for base, cls in (
        ("USB1_D", "USB_90"), ("USBHS_D", "USB_90"), ("USB2_D", "USB_90"),
        ("USB3_D", "USB_90"), ("USJ_D", "USB_90"),
        *((f"{pfx}_{lane}", "MIPI_100") for pfx in ("CSI", "CAM", "DSI", "DISP")
          for lane in ("D0", "D1", "CLK")),
        ("ETH_TX", "ETH_100"), ("ETH_RX", "ETH_100"),
    )
)


def is_keepout_exempt(comp: Component) -> bool:
    """Only the SoC and 0402 passives may sit inside the heatsink keep-out."""
    return comp.ref == "U1" or (comp.part.kind in ("res", "cap") and comp.part.package == "0402")


def validate() -> list[str]:
    """Return a list of design-rule problems (empty == clean)."""
    errors: list[str] = []
    # 1. unique refs
    dup = [r for r, n in Counter(c.ref for c in COMPONENTS).items() if n > 1]
    if dup:
        errors.append(f"duplicate refs: {dup}")
    # 2. pads exist
    for comp in COMPONENTS:
        bad = set(comp.conns) - comp.part.pad_numbers
        if bad:
            errors.append(f"{comp.ref}: unknown pads {sorted(bad)}")
    # 3. GPIO uniqueness + IO_MUX legality
    gpios = Counter(u.gpio for u in GPIO_MAP)
    errors += [f"GPIO{g} assigned {n}x" for g, n in gpios.items() if n > 1]
    overlap = set(gpios) & set(GPIO_NC)
    if overlap:
        errors.append(f"GPIOs both used and NC: {sorted(overlap)}")
    missing = set(range(55)) - set(gpios) - set(GPIO_NC)
    if missing:
        errors.append(f"GPIOs with no decision: {sorted(missing)}")
    iomux_net = {
        "RMII_REF_CLK": "RMII_REF_CLK", "RMII_TX_EN": "RMII_TX_EN", "RMII_TXD0": "RMII_TXD0",
        "RMII_TXD1": "RMII_TXD1", "RMII_CRS_DV": "RMII_CRS_DV", "RMII_RXD0": "RMII_RXD0",
        "RMII_RXD1": "RMII_RXD1", "SD0_CLK": "SD_CLK", "SD0_CMD": "SD_CMD", "SD0_D0": "SD_D0",
        "SD0_D1": "SD_D1", "SD0_D2": "SD_D2", "SD0_D3": "SD_D3", "U0TXD": "UART0_TXD",
        "U0RXD": "UART0_RXD", "USJ_DM": "USJ_DM", "USJ_DP": "USJ_DP",
        "LP_U0TXD": "LP_UART_TXD", "LP_U0RXD": "LP_UART_RXD",
    }
    by_net = {u.net: u.gpio for u in GPIO_MAP}
    for sig, net in iomux_net.items():
        if by_net.get(net) not in p4.IOMUX_OPTIONS[sig]:
            errors.append(f"{sig} on GPIO{by_net.get(net)} not in {p4.IOMUX_OPTIONS[sig]}")
    # 4. every SoC pad decided
    undecided = {p for p, _, _ in p4.PADS} - set(U1.conns) - {p4.GPIO_PAD[g] for g in GPIO_NC}
    if undecided:
        errors.append(f"U1 pads without net or NC decision: {sorted(undecided, key=int)}")
    # 5. nets with a single connection
    for net, members in nets().items():
        if len(members) < 2:
            errors.append(f"net {net} has a single connection {members}")
    # 6. strapping safety: GPIO35 needs a pull-up and nothing may pull GPIO36 low
    txd1 = [c for c in COMPONENTS if c.part.kind == "res" and set(c.conns.values()) == {"+3V3", "RMII_TXD1"}]
    if not txd1:
        errors.append("GPIO35 (boot strap) has no pull-up")
    s36 = [c for c in COMPONENTS if len(c.conns) == 2
           and set(c.conns.values()) == {"STRAP_GPIO36", GND}]
    if s36:
        errors.append("GPIO36 must not be pulled low")
    # 7. placement: every component placed; absolute placements obey keep-out
    for comp in COMPONENTS:
        if comp.place is None:
            errors.append(f"{comp.ref} has no placement")
        elif isinstance(comp.place, Place) and not is_keepout_exempt(comp):
            if abs(comp.place.x) < KEEPOUT_HALF and abs(comp.place.y) < KEEPOUT_HALF:
                errors.append(f"{comp.ref} ({comp.part.package}) placed inside heatsink keep-out")
    # 7b. nothing (origin-based) inside a heatsink standoff keep-out circle
    for comp in COMPONENTS:
        if comp.part.kind == "mech" or not isinstance(comp.place, Place):
            continue
        for hole, (hx, hy) in HEATSINK_HOLES.items():
            if ((comp.place.x - hx) ** 2 + (comp.place.y - hy) ** 2) ** 0.5 < HEATSINK_HOLE_KEEPOUT_R + 1.0:
                errors.append(f"{comp.ref} placed within {HEATSINK_HOLE_KEEPOUT_R + 1.0} mm of {hole}")
    # 8. decoupling must be 0402 (so it is legal inside the keep-out)
    for comp in COMPONENTS:
        if isinstance(comp.place, Near) and comp.place.ref == "U1" and not is_keepout_exempt(comp):
            errors.append(f"{comp.ref} is placed near U1 but is not an 0402 passive")
    return errors


def bom_lines() -> list[dict[str, object]]:
    """Group fitted parts by MPN (DNP parts are listed separately)."""
    groups: dict[tuple[str, bool], list[Component]] = defaultdict(list)
    for comp in COMPONENTS:
        if comp.part.kind == "mech":
            continue
        groups[(comp.part.key, comp.dnp)].append(comp)

    def refkey(ref: str) -> tuple[str, int]:
        m = re.match(r"([A-Z_]+?)(\d+)$", ref)
        return (m.group(1), int(m.group(2))) if m else (ref, 0)

    lines = []
    for (key, dnp), comps in sorted(groups.items(), key=lambda kv: (kv[0][1], refkey(kv[1][0].ref))):
        part = PARTS[key]
        refs = sorted((c.ref for c in comps), key=refkey)
        lines.append({"refs": refs, "qty": len(refs), "value": comps[0].value, "part": part,
                      "dnp": dnp})
    return lines


if __name__ == "__main__":  # pragma: no cover
    import sys

    problems = validate()
    print(f"{len(COMPONENTS)} components, {len(nets())} nets, {len(PARTS)} part types")
    print("\n".join(problems) if problems else "validate(): clean")
    sys.exit(1 if problems else 0)
