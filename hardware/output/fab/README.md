# ESP32-P4 Extreme -- fabrication package

> **Prototype files, routed by the project's own router (`tools/pcb_router.py`).**
> **Not fabricated or bench-tested yet.**
> The copper is fully connected and passes KiCad's DRC for manufacturing and electrical
> rules (0 errors).
> Some high-speed rules listed at the end are **not** met (coupling, gap or length
> matching of some differential pairs / length groups). Review those nets before
> ordering: they can limit USB 2.0 High-Speed, MIPI CSI/DSI or Ethernet margins.

## Files

- `esp32p4_extreme_gerbers.zip`: Gerber X2 (Protel extensions) + Excellon drills (PTH and NPTH separate) + drill maps + Gerber job file. Upload this to PCBWay.
- `esp32p4_extreme.ipc`: IPC-D-356 netlist for the fab's electrical test.
- `pick_and_place.csv`: centroids, both sides, origin = SoC centre (same as the Gerbers).
- `bom_pcbway.csv`: turnkey BOM (MPN, manufacturer, LCSC).
- Source: `hardware/output/routed/esp32p4_extreme.kicad_pcb` (KiCad 10).

## Board

- 110 x 80 mm, 1.6 mm, 4 layers, 234 footprints, 3223 track segments.
- Stack-up (1+2+1 HDI, controlled impedance 50 / 90 / 100 ohm, ask PCBWay to confirm widths):

  | Layer | Thickness | Material | Function |
  |---|---|---|---|
  | F.Cu | 0.035 mm | Cu 0.5 oz + plating | Signal_Top (HS routing, refs L2 GND) |
  | PP1 | 0.1 mm | Prepreg 1080 (laser-drillable) | Er 4 |
  | In1.Cu | 0.035 mm | Cu 1 oz | GND (solid, un-split reference plane) |
  | Core | 1.26 mm | FR-4 core, Tg170 | Er 4.4 |
  | In2.Cu | 0.035 mm | Cu 1 oz | VCC_3V3 plane (+VDD_HP island under U1) |
  | PP2 | 0.1 mm | Prepreg 1080 (laser-drillable) | Er 4 |
  | B.Cu | 0.035 mm | Cu 0.5 oz + plating | Signal_Bottom (low-speed, heat-spreader pour) |

- Vias: 230 x micro 0.3/0.1 mm, 688 x through 0.45/0.2 mm, 49 x through 0.6/0.3 mm.
  Laser microvias L1-L2 (to the GND plane) and L4-L3 (to the power planes) need an HDI
  1+2+1 build. Every via inside a pad (the EPAD array and the plane vias of the 0402 pads)
  must be **filled and capped** (IPC-4761 Type VII, "via in pad" / VIPPO).
- Minimum track 0.1 mm (0.09 mm only in the SoC fan-out, inside U1's courtyard), minimum clearance 0.09 mm there, 0.10 mm elsewhere.
- Surface finish ENIG (0.35 mm-pitch QFN), green mask, white silkscreen.

## Design-rule check (KiCad 10)

- Unconnected items: 0; manufacturing/electrical errors: 0.
- High-speed rules **not met**:

  - diff_pair_gap_out_of_range: CAM_D1_N, CAM_D1_P (2x)
  - diff_pair_gap_out_of_range: CSI_D1_N, CSI_D1_P (6x)
  - diff_pair_gap_out_of_range: DISP_CLK_N, DISP_CLK_P (2x)
  - diff_pair_gap_out_of_range: DISP_D0_N, DISP_D0_P (10x)
  - diff_pair_gap_out_of_range: DISP_D1_N, DISP_D1_P (22x)
  - diff_pair_gap_out_of_range: ETH_TX_N, ETH_TX_P (10x)
  - diff_pair_gap_out_of_range: USB1_D_N, USB1_D_P (44x)
  - diff_pair_gap_out_of_range: USB2_D_N, USB2_D_P (28x)
  - diff_pair_gap_out_of_range: USB3_D_N, USB3_D_P (71x)
  - diff_pair_gap_out_of_range: USBHS_D_N, USBHS_D_P (4x)
  - diff_pair_uncoupled_length_too_long: CAM_CLK_N, CAM_CLK_P (1x)
  - diff_pair_uncoupled_length_too_long: CAM_D0_N, CAM_D0_P (1x)
  - diff_pair_uncoupled_length_too_long: CAM_D1_N, CAM_D1_P (1x)
  - diff_pair_uncoupled_length_too_long: CSI_CLK_N, CSI_CLK_P (1x)
  - diff_pair_uncoupled_length_too_long: CSI_D0_N, CSI_D0_P (1x)
  - diff_pair_uncoupled_length_too_long: CSI_D1_N, CSI_D1_P (1x)
  - diff_pair_uncoupled_length_too_long: DISP_CLK_N, DISP_CLK_P (1x)
  - diff_pair_uncoupled_length_too_long: DISP_D0_N, DISP_D0_P (1x)
  - diff_pair_uncoupled_length_too_long: DISP_D1_N, DISP_D1_P (1x)
  - diff_pair_uncoupled_length_too_long: DSI_CLK_N, DSI_CLK_P (1x)
  - diff_pair_uncoupled_length_too_long: DSI_D0_N, DSI_D0_P (1x)
  - diff_pair_uncoupled_length_too_long: DSI_D1_N, DSI_D1_P (1x)
  - diff_pair_uncoupled_length_too_long: ETH_RX_N, ETH_RX_P (1x)
  - diff_pair_uncoupled_length_too_long: ETH_TX_N, ETH_TX_P (1x)
  - diff_pair_uncoupled_length_too_long: USB1_D_N, USB1_D_P (1x)
  - diff_pair_uncoupled_length_too_long: USB2_D_N, USB2_D_P (1x)
  - diff_pair_uncoupled_length_too_long: USB3_D_N, USB3_D_P (1x)
  - diff_pair_uncoupled_length_too_long: USBHS_D_N, USBHS_D_P (1x)
  - skew_out_of_range: CAM_CLK_N (1x)
  - skew_out_of_range: CAM_D0_N (1x)
  - skew_out_of_range: CAM_D0_P (1x)
  - skew_out_of_range: CAM_D1_N (1x)
  - skew_out_of_range: CAM_D1_P (1x)
  - skew_out_of_range: CSI_CLK_N (1x)
  - skew_out_of_range: CSI_CLK_P (1x)
  - skew_out_of_range: CSI_D0_N (1x)
  - skew_out_of_range: CSI_D0_P (1x)
  - skew_out_of_range: CSI_D1_P (1x)
  - skew_out_of_range: DISP_CLK_N (1x)
  - skew_out_of_range: DISP_CLK_P (1x)
  - skew_out_of_range: DISP_D0_N (1x)
  - skew_out_of_range: DISP_D1_N (1x)
  - skew_out_of_range: DISP_D1_P (1x)
  - skew_out_of_range: DSI_CLK_P (1x)
  - skew_out_of_range: DSI_D0_N (1x)
  - skew_out_of_range: DSI_D0_P (1x)
  - skew_out_of_range: DSI_D1_N (1x)
  - skew_out_of_range: DSI_D1_P (1x)
  - skew_out_of_range: ETH_RX_N (1x)
  - skew_out_of_range: ETH_TX_P (1x)
  - skew_out_of_range: SD_CLK_C (1x)
  - skew_out_of_range: SD_CMD (1x)
  - skew_out_of_range: SD_CMD_C (1x)
  - skew_out_of_range: SD_D0 (1x)
  - skew_out_of_range: SD_D0_C (1x)
  - skew_out_of_range: SD_D1 (1x)
  - skew_out_of_range: SD_D1_C (1x)
  - skew_out_of_range: SD_D2 (1x)
  - skew_out_of_range: SD_D3 (1x)
  - skew_out_of_range: USB2_D_P (1x)
  - skew_out_of_range: USB3_D_P (1x)
  - skew_out_of_range: USBHS_D_N (1x)

