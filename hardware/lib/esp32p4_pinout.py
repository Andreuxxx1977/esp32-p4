"""ESP32-P4X (chip rev v3.x) QFN-104 pad data -- the silicon ground truth.

Every value in this module comes from an Espressif primary source, so the
rest of the design can be checked against it:

* Pad numbers / names / electrical types: Espressif KiCad library
  ``symbols/Espressif.kicad_sym`` -> symbol ``ESP32-P4X``
  (keywords ESP32-P4NRW16X, ESP32-P4NRW32X). Pad 105 is the exposed pad (GND).
* Land pattern: Espressif ``footprints/Espressif.pretty/ESP32-P4.kicad_mod``
  (0.20 x 0.65 mm pads, 0.35 mm pitch, pad rows at +/-4.875 mm,
  EPAD = 3 x 3 grid of 2.1 mm tiles on a 2.7 mm pitch -> 7.5 x 7.5 mm).
* Power domain / reset state per GPIO: esp-hardware-design-guidelines
  ``docs/en/esp32p4/esp32p4-table-io-mux.inc``.
* IO_MUX-fixed peripheral pads: ESP-IDF ``components/soc/esp32p4/emac_periph.c``,
  ``soc/sdmmc_pins.h``, ``soc/uart_pins.h`` and the P4 schematic checklist.
"""

from __future__ import annotations

# (pad, name, KiCad electrical type) -- verbatim from Espressif ESP32-P4X symbol.
PADS: tuple[tuple[str, str, str], ...] = (
    ("1", "GPIO1", "bidirectional"),
    ("2", "GPIO2", "bidirectional"),
    ("3", "GPIO3", "bidirectional"),
    ("4", "GPIO4", "bidirectional"),
    ("5", "GPIO5", "bidirectional"),
    ("6", "GPIO6", "bidirectional"),
    ("7", "GPIO7", "bidirectional"),
    ("8", "GPIO8", "bidirectional"),
    ("9", "VDD_LP", "power_in"),
    ("10", "GPIO9", "bidirectional"),
    ("11", "GPIO10", "bidirectional"),
    ("12", "GPIO11", "bidirectional"),
    ("13", "GPIO12", "bidirectional"),
    ("14", "GPIO13", "bidirectional"),
    ("15", "GPIO14", "bidirectional"),
    ("16", "GPIO15", "bidirectional"),
    ("17", "GPIO16/ADC1_CHANNEL0", "bidirectional"),
    ("18", "GPIO17/ADC1_CHANNEL1", "bidirectional"),
    ("19", "GPIO18/ADC1_CHANNEL2", "bidirectional"),
    ("20", "GPIO19/ADC1_CHANNEL3", "bidirectional"),
    ("21", "VDD_IO_0", "power_in"),
    ("22", "GPIO20/ADC1_CHANNEL4", "bidirectional"),
    ("23", "GPIO21/ADC1_CHANNEL5", "bidirectional"),
    ("24", "GPIO22/ADC1_CHANNEL6", "bidirectional"),
    ("25", "GPIO23/ADC1_CHANNEL7", "bidirectional"),
    ("26", "VDD_HP_0", "power_in"),
    ("27", "FLASH_CS", "bidirectional"),
    ("28", "FLASH_Q", "bidirectional"),
    ("29", "FLASH_WP", "bidirectional"),
    ("30", "VDD_FLASH_IO", "power_in"),
    ("31", "FLASH_HOLD", "bidirectional"),
    ("32", "FLASH_CK", "bidirectional"),
    ("33", "FLASH_D", "bidirectional"),
    ("34", "DSI_REXT", "bidirectional"),
    ("35", "DSI_DATAP1", "bidirectional"),
    ("36", "DSI_DATAN1", "bidirectional"),
    ("37", "DSI_CLKN", "bidirectional"),
    ("38", "DSI_CLKP", "bidirectional"),
    ("39", "DSI_DATAP0", "bidirectional"),
    ("40", "DSI_DATAN0", "bidirectional"),
    ("41", "VDD_MIPI_DPHY", "power_in"),
    ("42", "CSI_DATAN0", "bidirectional"),
    ("43", "CSI_DATAP0", "bidirectional"),
    ("44", "CSI_CLKP", "bidirectional"),
    ("45", "CSI_CLKN", "bidirectional"),
    ("46", "CSI_DATAN1", "bidirectional"),
    ("47", "CSI_DATAP1", "bidirectional"),
    ("48", "CSI_REXT", "bidirectional"),
    ("49", "USB-DM", "bidirectional"),
    ("50", "USB-DP", "bidirectional"),
    ("51", "VDD_USBPHY", "power_in"),
    ("52", "GPIO24/USB1P1_N0", "bidirectional"),
    ("53", "GPIO25/USB1P1_P0", "bidirectional"),
    ("54", "VDD_HP_1", "power_in"),
    ("55", "GPIO26/USB1P1_N1", "bidirectional"),
    ("56", "GPIO27/USB1P1_P1", "bidirectional"),
    ("57", "GPIO28", "bidirectional"),
    ("58", "GPIO29", "bidirectional"),
    ("59", "VDD_PSRAM_0", "power_in"),
    ("60", "GPIO30", "bidirectional"),
    ("61", "GPIO31", "bidirectional"),
    ("62", "VDD_IO_4", "power_in"),
    ("63", "GPIO32", "bidirectional"),
    ("64", "GPIO33", "bidirectional"),
    ("65", "GPIO34", "bidirectional"),
    ("66", "GPIO35", "bidirectional"),
    ("67", "VDD_PSRAM_1", "power_in"),
    ("68", "GPIO36", "bidirectional"),
    ("69", "GPIO37", "bidirectional"),
    ("70", "GPIO38", "bidirectional"),
    ("71", "VDDO_FLASH", "power_out"),
    ("72", "VDDO_PSRAM", "power_out"),
    ("73", "VDDO_3", "power_out"),
    ("74", "VDDO_4", "power_out"),
    ("75", "VDD_LDO", "power_in"),
    ("76", "VDD_HP_2", "power_in"),
    ("77", "VDD_DCDCC", "power_in"),
    ("78", "FB_DCDC", "bidirectional"),
    ("79", "EN_DCDC", "output"),
    ("80", "GPIO39", "bidirectional"),
    ("81", "GPIO40", "bidirectional"),
    ("82", "GPIO41", "bidirectional"),
    ("83", "GPIO42", "bidirectional"),
    ("84", "GPIO43", "bidirectional"),
    ("85", "VDD_IO_5", "power_in"),
    ("86", "GPIO44", "bidirectional"),
    ("87", "GPIO45", "bidirectional"),
    ("88", "GPIO46", "bidirectional"),
    ("89", "GPIO47", "bidirectional"),
    ("90", "GPIO48", "bidirectional"),
    ("91", "VDD_HP_3", "power_in"),
    ("92", "GPIO49/ADC2_CHANNEL0", "bidirectional"),
    ("93", "GPIO50/ADC2_CHANNEL1", "bidirectional"),
    ("94", "GPIO51/ADC2_CHANNEL2", "bidirectional"),
    ("95", "GPIO52/ADC2_CHANNEL3", "bidirectional"),
    ("96", "VDD_IO_6", "power_in"),
    ("97", "GPIO53/ADC2_CHANNEL4", "bidirectional"),
    ("98", "GPIO54/ADC2_CHANNEL5", "bidirectional"),
    ("99", "XTAL_N", "bidirectional"),
    ("100", "XTAL_P", "bidirectional"),
    ("101", "VDD_ANA", "bidirectional"),
    ("102", "VDD_BAT", "bidirectional"),
    ("103", "CHIP_PU", "input"),
    ("104", "GPIO0", "bidirectional"),
    ("105", "GND", "power_in"),
)

PAD_NAME: dict[str, str] = {p: n for p, n, _ in PADS}
PAD_TYPE: dict[str, str] = {p: t for p, _, t in PADS}

# ---------------------------------------------------------------------------
# Land-pattern geometry (Espressif ESP32-P4.kicad_mod, footprint origin = body
# centre, KiCad axes: +X right, +Y down, rotation 0).
# ---------------------------------------------------------------------------
PITCH_MM = 0.35
ROW_OFFSET_MM = 4.875          # pad-centre distance from body centre
PAD_SIZE_MM = (0.20, 0.65)     # (width along the row, length toward the body)
BODY_MM = (10.0, 10.0)
HEIGHT_MAX_MM = 0.90           # QFN-104 seated height used for the heatsink stack
EPAD_SIZE_MM = 7.5             # outer extent of the 3 x 3 EPAD tiles
EPAD_TILE_MM = 2.1
EPAD_TILE_PITCH_MM = 2.7


def pad_position(pad: str | int) -> tuple[float, float]:
    """Return the (x, y) centre of a perimeter pad or EPAD in footprint mm."""
    n = int(pad)
    first = -4.375
    if 1 <= n <= 26:      # left edge, top -> bottom
        return (-ROW_OFFSET_MM, first + (n - 1) * PITCH_MM)
    if 27 <= n <= 52:     # bottom edge, left -> right
        return (first + (n - 27) * PITCH_MM, ROW_OFFSET_MM)
    if 53 <= n <= 78:     # right edge, bottom -> top
        return (ROW_OFFSET_MM, -first - (n - 53) * PITCH_MM)
    if 79 <= n <= 104:    # top edge, right -> left
        return (-first - (n - 79) * PITCH_MM, -ROW_OFFSET_MM)
    if n == 105:
        return (0.0, 0.0)
    raise ValueError(f"no pad {pad}")


def pad_side(pad: str | int) -> str:
    n = int(pad)
    if n == 105:
        return "center"
    return ("left", "bottom", "right", "top")[(n - 1) // 26]


# ---------------------------------------------------------------------------
# GPIO facts (esp32p4-table-io-mux.inc)
# ---------------------------------------------------------------------------
GPIO_PAD: dict[int, str] = {}
for _pad, _name, _ in PADS:
    if _name.startswith("GPIO"):
        GPIO_PAD[int(_name.split("/")[0][4:])] = _pad
assert sorted(GPIO_PAD) == list(range(55)), "ESP32-P4 has GPIO0..GPIO54"


def gpio_domain(gpio: int) -> str:
    """IO power-supply pin feeding this GPIO's pad ring segment."""
    if gpio in (0, 1, 2, 3):
        return "VDD_LP/VDD_BAT"
    if gpio <= 15:
        return "VDD_LP"
    if gpio <= 23:
        return "VDD_IO_0"
    if gpio <= 38:
        return "VDD_IO_4"
    if gpio <= 48:
        return "VDD_IO_5"
    return "VDD_IO_6"


# Default pad state straight after reset (only non-'--' entries listed).
RESET_STATE: dict[int, str] = {
    2: "IE, WPU (after reset)",
    3: "IE (after reset)",
    4: "IE (after reset)",
    25: "IE, USB_PU (after reset)",
    32: "IE (at reset)",
    33: "IE (at reset)",
    34: "IE (at reset)",
    35: "IE, WPU (at reset)",
    36: "IE (at reset)",
    37: "IE (at reset)",
    38: "IE (at reset)",
}

# Strapping pins (schematic checklist, "Strapping Pins").
# Boot mode: GPIO35=1 -> SPI boot; GPIO35=0 & GPIO36=1 -> joint download boot.
STRAPPING_GPIOS = frozenset({34, 35, 36, 37, 38})
BOOT_MODE_GPIO = 35
DOWNLOAD_QUALIFIER_GPIO = 36

ADC_CHANNEL: dict[int, str] = {
    16: "ADC1_CH0", 17: "ADC1_CH1", 18: "ADC1_CH2", 19: "ADC1_CH3",
    20: "ADC1_CH4", 21: "ADC1_CH5", 22: "ADC1_CH6", 23: "ADC1_CH7",
    49: "ADC2_CH0", 50: "ADC2_CH1", 51: "ADC2_CH2", 52: "ADC2_CH3",
    53: "ADC2_CH4", 54: "ADC2_CH5",
}

# ---------------------------------------------------------------------------
# IO_MUX-restricted peripheral signals: signal -> legal GPIOs.
# A design that routes these signals anywhere else will not work.
# ---------------------------------------------------------------------------
IOMUX_OPTIONS: dict[str, tuple[int, ...]] = {
    # EMAC RMII (emac_periph.c / esp32p4-emac.inc). REF_CLK is input-only.
    "RMII_REF_CLK": (32, 44, 50),
    "RMII_TX_EN": (33, 40, 49),
    "RMII_TXD0": (34, 41),
    "RMII_TXD1": (35, 42),
    "RMII_CRS_DV": (28, 45, 51),
    "RMII_RXD0": (29, 46, 52),
    "RMII_RXD1": (30, 47, 53),
    "RMII_RX_ER": (31, 48, 54),
    "RMII_TX_ER": (36, 43),
    "REF_50M_CLK_OUT": (23, 39),
    # SDMMC host slot 0 (UHS-I capable, IO_MUX only) -- sdmmc_pins.h
    "SD0_CLK": (43,), "SD0_CMD": (44,),
    "SD0_D0": (39,), "SD0_D1": (40,), "SD0_D2": (41,), "SD0_D3": (42,),
    "SD0_D4": (45,), "SD0_D5": (46,), "SD0_D6": (47,), "SD0_D7": (48,),
    # UART0 (ROM download / console) -- uart_pins.h
    "U0TXD": (37,), "U0RXD": (38,),
    # LP UART (schematic checklist: LP UART TXD = LP GPIO14, RXD = LP GPIO15)
    "LP_U0TXD": (14,), "LP_U0RXD": (15,),
    # USB Serial/JTAG full-speed PHY (default pads; roles can be swapped)
    "USJ_DM": (24,), "USJ_DP": (25,),
    # USB 1.1 FS OTG default pads
    "USB_FS_OTG_DM": (26,), "USB_FS_OTG_DP": (27,),
}

# Functions on dedicated (non-GPIO) pads. These can never collide with a GPIO.
DEDICATED_PADS: dict[str, str] = {
    "MIPI DSI (2-lane D-PHY)": "34-40",
    "MIPI D-PHY supply": "41",
    "MIPI CSI (2-lane D-PHY)": "42-48",
    "USB 2.0 HS OTG UTMI PHY": "49-51",
    "Quad-SPI flash MSPI": "27-33",
    "40 MHz crystal": "99-100",
    "External VDD_HP DCDC control": "78-79",
}
