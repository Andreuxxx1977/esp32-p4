# Contributing

Thanks for helping! This board is described by **data, not drawings**. One Python module,
[`hardware/lib/board_spec.py`](hardware/lib/board_spec.py), holds every part, connection and placement.
The netlist, BOM, pinout table and docs are all generated from it.

## The golden rule

**Edit `hardware/lib/board_spec.py` (or `esp32p4_pinout.py` / `impedance.py`), then regenerate.
Never hand-edit generated files:**

| Generated file | Produced by |
|---|---|
| `hardware/output/esp32p4_extreme.net`, `esp32p4_extreme_nets.csv`, `esp32p4_extreme_erc.log` | `python -m hardware.skidl.esp32p4_extreme_netlist` |
| `docs/TASK1_pinout.md`, `docs/TASK2_mechanical_thermal.md`, `docs/TASK4_bom_pcbway.md`, `hardware/output/bom_pcbway.csv` | `python -m tools.gen_docs` |

Commit the regenerated files **in the same commit** as the spec change. CI fails if they are stale.

## Workflow

```bash
pip install --use-pep517 -r requirements.txt

# 1. edit hardware/lib/board_spec.py
python -m hardware.lib.board_spec                  # 2. design-rule validation (must print "clean")
python -m hardware.skidl.esp32p4_extreme_netlist   # 3. netlist; refuses to write if ERC is not clean
python -m hardware.skidl.verify_netlist            # 4. netlist == spec
python -m hardware.skidl.check_footprints          # 5. if you touched parts/footprints (needs internet)
python -m tools.gen_docs                           # 6. docs + BOM
python -m pytest -q                                # 7. all tests green
```

## Conventions in the data model

- **Coordinates:** millimetres, relative to the **ESP32-P4 body centre = (0, 0)**, KiCad axes
  (+X right, **+Y down**). `Place(x, y, rot)` is the footprint origin; for pin headers that is pin 1.
- **Relative placement:** `Near(ref, pad, max_mm)` means "place within `max_mm` of that pad". The SoC
  decoupling uses `max_mm = 2.0`.
- **Heatsink keep-out:** inside the 25 x 25 mm square around the SoC only **U1 and 0402 passives** are
  allowed (the heatsink sits flush on the SoC). Also keep parts `HEATSINK_HOLE_KEEPOUT_R` (3.5 mm)
  away from the four M2.5 standoff holes. `validate()` enforces both.
- **Pins are pad numbers.** `PartType.pins` must use the pad numbers of the real KiCad footprint
  (e.g. USB-C shell = `SH`, FPC mounting pads = `MP`). Take pin maps from the
  [official KiCad symbol library](https://gitlab.com/kicad/libraries/kicad-symbols) where one exists,
  record where they came from in `pin_source`, and run `check_footprints`.
- **GPIOs:** every GPIO0..54 must appear exactly once in `GPIO_MAP` or `GPIO_NC`. Signals that the
  ESP32-P4 can only route through IO_MUX (RMII, SDMMC slot 0, UART0, USB-Serial-JTAG, LP-UART) must
  stay on a pad listed in `esp32p4_pinout.IOMUX_OPTIONS`. The tests check this.
- **Strapping pins** GPIO34-38: GPIO35 needs its pull-up (SPI boot). Nothing may pull GPIO36 low.
- **New parts:** give a real, orderable MPN, the manufacturer and (if available) the LCSC number, and
  add the evidence to [`docs/component_verification.md`](docs/component_verification.md).

## Where help is most welcome

1. **Routing** the placed board in KiCad 9/10 while respecting the impedance rules
   (`hardware/lib/impedance.py`: 50 ohm SE, 90 ohm USB, 100 ohm MIPI/Ethernet) and the Espressif
   layout rules summarised in `docs/TASK1_pinout.md`.
2. **Datasheet review** of the items still marked **UNVERIFIED** in `docs/component_verification.md`.
3. **Fabrication and bring-up reports**: photos, current measurements, thermal readings.
4. **Firmware**: ESP-IDF board support (fan curve from the internal temperature sensor, LAN8720A, SD UHS-I).

## Reporting problems

Open an issue with the reference designator(s) and net names involved (e.g. `U5 pad 14, net
RMII_REF_CLK_PHY`), what you expected, and a source (datasheet page, Espressif guideline) if you have one.

## Licence of contributions

By contributing you agree that your contribution is licensed under the project's licence,
**CERN-OHL-P-2.0** (see [`LICENSE`](LICENSE)). Please keep the attribution notice in the README.
