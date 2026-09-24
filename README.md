# ESP32-P4 "Extreme Performance" development board

Actively cooled ESP32-P4 board for heavy multimedia workloads: 2-lane MIPI CSI + 2-lane MIPI DSI,
USB 2.0 High-Speed, 10/100 Ethernet (RMII), UHS-I-capable microSD, 32 MB Quad-SPI flash and 32 MB
in-package PSRAM, on a 4-layer HDI board with a 3 A 3.3 V rail.

Everything (pinout table, SKiDL netlist, pcbnew placement, BOM, tests) is generated from one Python
data model, so the four deliverables cannot drift apart:

```
hardware/lib/esp32p4_pinout.py   ESP32-P4X pad table + IO_MUX facts (from Espressif sources)
hardware/lib/board_spec.py       parts, MPNs, nets, GPIO map, placement, mechanics   <- source of truth
hardware/lib/impedance.py        stack-up + controlled-impedance geometry
hardware/skidl/                  TASK 3 - SKiDL netlist generator (+ netlist/footprint verifiers)
hardware/pcbnew/                 TASK 3 - pcbnew placement script, custom DRC rules
tools/gen_docs.py                regenerates docs/ and the BOM CSV from the spec
docs/TASK1_pinout.md             TASK 1 - architecture + GPIO pinout + conflict analysis
docs/TASK2_mechanical_thermal.md TASK 2 - thermal/airflow strategy + coordinates + collision check
docs/TASK4_bom_pcbway.md         TASK 4 - PCBWay turnkey BOM (also hardware/output/bom_pcbway.csv)
tests/                           pytest: pin conflicts, strapping, keep-out, layout, BOM consistency
```

## Quick start

```bash
python3 -m hardware.lib.board_spec                 # validate the design data
pip install skidl pytest
python3 -m hardware.skidl.esp32p4_extreme_netlist  # -> hardware/output/esp32p4_extreme.net
python3 -m tools.gen_docs                          # -> docs/*.md, hardware/output/bom_pcbway.csv
python3 -m pytest -q
# Inside KiCad 8/9 (pcbnew Python):
python3 hardware/pcbnew/esp32p4_extreme_place.py --out hardware/output/esp32p4_extreme.kicad_pcb
```

## Architecture decisions that deviate from the brief

These changes come from Espressif's primary sources: the ESP-IDF `soc_caps.h`/`*_pins.h`/`emac_periph.c`
files, the ESP32-P4 hardware design guidelines and Espressif's KiCad library.

| Brief | Built | Why |
|---|---|---|
| ESP32-P4NR32 | **ESP32-P4NRW32X** (rev v3.x, 32 MB PSRAM) | Orderable name of the 32 MB-PSRAM part. Espressif marks rev v1.x (no "X") as not recommended for new designs. |
| 32 MB **Octal** flash (MX25UM25645G) | 32 MB **Quad** W25Q256JVEIQ (3.3 V) | The P4 flash MSPI supports SPI/Dual/Quad only, up to 64 MB. ESP-IDF defines no `SOC_SPI_MEM_SUPPORT_FLASH_OPI_MODE` for the P4 (the S3 defines it). MX25UM is also a 1.8 V part, while `VDDO_FLASH` defaults to 3.3 V. |
| MP2315 / SY8113B buck | **TPS62130** (3-17 V, 3 A, 100 % duty, QFN with EP) | Runs from VBUS after Schottky OR-ing (~4.4-4.6 V), below the MP2315's 4.5 V minimum. The exposed pad also gives a better thermal path at 3 A continuous. |
| (not in brief) | **TLV62569** VDD_HP DCDC | The P4 core rail (0.99-1.3 V) must come from an external DCDC that the SoC itself enables and trims (`EN_DCDC`/`FB_DCDC`). TLV62569 is on Espressif's verified list. |
| RTL8201F | **LAN8720A** | Pinout verified from the official KiCad library, ESP-IDF `lan87xx` driver, REF_CLK-out mode feeds the P4's input-only RMII clock. |
| CH340K / CP2102N "UART/JTAG" | **CP2102N** (UART + auto-program) **plus** a 3rd USB-C on the P4's native USB-Serial-JTAG | Neither CH340K nor CP2102N can do JTAG. The P4's built-in USB-Serial-JTAG (GPIO24/25) gives JTAG and download with no setup. |
| AO3400A fan MOSFET | AO3400A as **open-drain PWM driver** + AO3401A high-side switch | A 4-wire fan needs constant power with an open-drain PWM input (Intel spec). Chopping its ground breaks the tach and the fan's controller. The high-side switch adds a zero-RPM mode, and the fan runs at 100 % if the firmware hangs. |

See `docs/TASK1_pinout.md` for the full GPIO map and the zero-conflict proof.

## License

Copyright (c) 2026 Andreuxxx1977.

This source describes Open Hardware and is licensed under the CERN-OHL-S v2 (SPDX: `CERN-OHL-S-2.0`).
The licence covers the design data, the scripts that generate the netlist, placement, BOM and docs, and
their outputs. The full text is in [`LICENSE`](LICENSE).

You may redistribute and modify this source and make products using it under the terms of the
CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.txt). This source is distributed WITHOUT ANY EXPRESS OR
IMPLIED WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR
PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.

Source location: https://github.com/Andreuxxx1977/esp32-p4

As per CERN-OHL-S v2 section 4, should You produce hardware based on this source, You must where
practicable maintain the Source Location visible on the external case of the product or other products
you make using this source. The placement script also prints the source location and licence on the
PCB silkscreen.
