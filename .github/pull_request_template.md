## What changes

<!-- Which blocks / parts / nets are affected and why. -->

## Checklist

- [ ] I edited `hardware/lib/board_spec.py` (or `esp32p4_pinout.py` / `impedance.py`), not the generated files
- [ ] `python -m hardware.lib.board_spec` prints `validate(): clean`
- [ ] Regenerated and committed the netlist (`python -m hardware.skidl.esp32p4_extreme_netlist`) and docs/BOM (`python -m tools.gen_docs`)
- [ ] `python -m pytest -q` passes
- [ ] New or changed MPNs have evidence in `docs/component_verification.md`
