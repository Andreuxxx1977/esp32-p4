> **Component verification record.** Evidence for every MPN, LCSC number and third-party pinout used in
> `hardware/lib/board_spec.py`. The candidates marked NOT FOUND / FAILS / CORRECTED-TO below have already been
> replaced in the spec (TPD2EUSB30DRTR on USB-HS, MF-MSMF250/16X-2, Amphenol F32Q FPC, CTS 402F4001XIAR,
> YXC X322525MOB4SI, TPD4E05U06DQAR on microSD). Items still marked UNVERIFIED should be confirmed against the
> datasheet before ordering.

# ESP32-P4 dev board: MPN verification and pinout report

Date: 2026-09-24. Methods: web search result titles and snippets from LCSC, JLCPCB, Mouser, DigiKey, Newark, RS, TI, ST, Microchip, SiLabs and Espressif; the official KiCad libraries on GitLab, fetched with curl, with pins parsed from the `.kicad_sym` files; ESP-IDF and raspberrypi/documentation sources on raw.githubusercontent.com. Vendor PDFs could not be opened because the proxy blocks them, so datasheet numbers come from search snippets. Anything I could not confirm is marked **UNVERIFIED**.

---

## 1. MPN verification table

| # | Candidate MPN | Status | Corrected / Final MPN | Key spec verified | LCSC C# | Evidence |
|---|---|---|---|---|---|---|
| 1 | ESP32-P4NRW32X | VERIFIED | ESP32-P4NRW32X | Chip rev v3.x, 32 MB in-package PSRAM, no in-package flash, QFN-104 10x10 | C54540373 (JLCPCB) | https://jlcpcb.com/partdetail/EspressifSystems-ESP32P4NRW32X/C54540373 ; https://documentation.espressif.com/esp32-p4-chip-revision-v3.x_user_guide_en.html |
| 1b | ESP32-P4NRW32 (no X) | VERIFIED as the older part | — | This is the v1.x part (see the rev v1.3 datasheet). It is marked EOL and replaced by the ...X part. v1.x and v3.x need separate firmware builds. | C22387510 | https://www.lcsc.com/product-detail/C22387510.html ; https://documentation.espressif.com/esp32-p4-chip-revision-v1.3_datasheet_en.pdf |
| 2 | W25Q256JVEIQ | VERIFIED | W25Q256JVEIQ | 256 Mbit, 3 V, SPI/Dual/Quad at 133 MHz, WSON-8 8x6 | C97522 | https://www.lcsc.com/product-detail/C97522.html ; https://www.digikey.com/en/products/detail/winbond-electronics/W25Q256JVEIQ/6810681 |
| 2b | MX25L25645GZ2I-08G | VERIFIED | (alternate) | 256 Mbit, 2.7-3.6 V, 120 MHz, WSON-8 8x6 | C2802978 | https://www.lcsc.com/product-detail/NOR-FLASH_MXIC-Macronix-MX25L25645GZ2I-08G_C2802978.html |
| 3 | TPS62130RGTR | VERIFIED | TPS62130RGTR | 3-17 V in, 3 A, 0.9-6 V out, VQFN-16 3x3, 2.5 MHz (see the FSW/DEF/VFB answers below the table) | C43590 | https://www.lcsc.com/product-detail/DC-DC-Converters_TI_TPS62130RGTR_TPS62130RGTR_C43590.html ; https://www.ti.com/lit/ds/symlink/tps62130.pdf |
| 4 | TLV62569DBVR | VERIFIED | TLV62569DBVR | 2 A buck, **VIN 2.5-5.5 V**, 1.5 MHz, SOT-23-5, VFB 0.6 V (KiCad: adjustable 0.6-5.5 V). The DBV package has no PG pin. Pins: 1 EN, 2 GND, 3 SW, 4 VIN, 5 FB | C141836 | https://www.lcsc.com/product-detail/C141836.html ; KiCad Regulator_Switching:TLV62569DBV |
| 5 | LAN8720A-CP-TR | VERIFIED | LAN8720A-CP-TR (0 to 85 °C). Industrial alternate: LAN8720AI-CP-TR, C17146 | 10/100 RMII PHY, QFN-24 4x4, 1.6-3.6 V VDDIO (strap answers below the table) | C45223 | https://www.lcsc.com/product-detail/C45223.html ; https://ww1.microchip.com/downloads/en/DeviceDoc/LAN8720A%20QFN%20Rev%20D%20Schematic%20Checklist.pdf |
| 6 | CP2102N-A02-GQFN24R | VERIFIED | CP2102N-A02-GQFN24R | USB 2.0 FS to UART, QFN-24 4x4 (answers below the table) | C969151 | https://www.lcsc.com/product-detail/C969151.html |
| 7 | USBLC6-2SC6 | VERIFIED, **but FAILS the ≤1 pF HS requirement** | For USB-HS: **TPD2EUSB30DRTR** (TI) | USBLC6-2SC6 I/O-GND capacitance is **3.5 pF max** (typ. around 2.5-3 pF per snippets, typ. UNVERIFIED). TPD2EUSB30: **0.7 pF typ**, 5.5 V working voltage (tolerates a short to VBUS), DRT-3 1x1 mm | USBLC6: C7519; TPD2EUSB30DRTR: C97502 (alt. TPD2EUSB30ADRTR C94934, 3.6 V working) | https://www.st.com/resource/en/datasheet/usblc6-2.pdf ; https://datasheet.lcsc.com/lcsc/1810231723_Texas-Instruments-TPD2EUSB30DRTR_C97502.pdf ; Espressif ≤1 pF: https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32p4/schematic-checklist-esp32p4.html |
| 8 | MMDT3904-7-F | VERIFIED | MMDT3904-7-F | Dual NPN, 40 V VCEO, 200 mA, 200 mW, SOT-363 | C83572 | https://www.lcsc.com/product-detail/bipolar-transistors-bjt_diodes-incorporated-mmdt3904-7-f_C83572.html |
| 9a | AO3400A | VERIFIED | AO3400A | N-ch 30 V 5.7 A, RDS(on) 26 mΩ at 4.5 V / 48 mΩ at 2.5 V, SOT-23 | C20917 | https://www.lcsc.com/product-detail/C20917.html |
| 9b | AO3401A | VERIFIED | AO3401A (AOS) | P-ch -30 V -4 A, SOT-23 | C15127 | https://lcsc.com/product-detail/MOSFET_AOS_AO3401A_AO3401A_C15127.html |
| 10 | DM3AT-SF-PEJM5 | VERIFIED | DM3AT-SF-PEJM5 | microSD, push-push, detect switch (2 pins, closes when a card is inserted), 1.68 mm height | C114218 | https://www.lcsc.com/product-detail/SD-Card-Memory-Card-Connector_HRS-Hirose-DM3AT-SF-PEJM5_C114218.html |
| 11 | FH12-22S-0.5SH(55) | VERIFIED, **BOTTOM contact** | See the design-impacting findings. The Raspberry Pi itself uses **Amphenol F32Q-1A7H1-11022 (TOP contact)** | 22 pos, 0.5 mm, ZIF, SMT right-angle, 2.0 mm high, bottom contact | C597981 (presales / out of stock). F32Q-1A7H1-11022: C3169253 | https://www.digikey.com/en/products/detail/hirose-electric-co-ltd/FH12-22S-0-5SH-55/1110321 ; https://www.lcsc.com/product-detail/C3169253.html ; https://uk.farnell.com/amphenol-icc/f32q-1a7h1-11022/conn-r-a-ffc-fpc-22pos-1row-0/dp/3526754 |
| 12 | Molex 47053-1000 | VERIFIED | 47053-1000 | KK 254, 4-pos, 2.54 mm, vertical THT, polarized fan header. The KiCad fan-header footprint descr names this MPN | C240840 | https://lcsc.com/product-detail/Others_MOLEX_47053-1000_MOLEX-47053-1000_C240840.html |
| 13 | USB4105-GF-A | VERIFIED | USB4105-GF-A | USB 2.0 Type-C, 16 contacts, horizontal top-mount, 5 A / 48 V | C3020560 | https://www.lcsc.com/product-detail/usb-connectors_global-connector-technology-usb4105-gf-a_C3020560.html |
| 14 | PMEG4050EP,115 | VERIFIED | PMEG4050EP,115 | 40 V 5 A, SOD128. VF at 25 °C: 340 mV typ / 390 mV max at 1 A, 430 mV typ / 490 mV max at 5 A. **At 2.5 A about 0.37-0.38 V typ, about 0.43 V max** (interpolated between those points; there is no datasheet row at 2.5 A). That is about 1 W dissipation at 2.5 A | C96235 (JLCPCB) | https://assets.nexperia.com/documents/data-sheet/PMEG4050EP.pdf ; https://jlcpcb.com/partdetail/Nexperia-PMEG4050EP115/C96235 |
| 15 | MF-MSMF250-2 | **NOT FOUND** | **CORRECTED-TO MF-MSMF250/16X-2** | 1812, I_hold 2.5 A, I_trip 5.0 A, **16 V** (≥6 V OK), I_max 100 A, R ≈ 15 mΩ. This part replaced MF-MSMF250/16-2 (Bourns PCN MF1205) | C210838 | https://www.lcsc.com/product-detail/C210838.html ; https://www.bourns.com/docs/product-datasheets/mf-msmf.pdf |
| 16 | BLM18PG221SN1D | VERIFIED | BLM18PG221SN1D | 220 Ω ±25 % at 100 MHz, 1.4 A, 0.1 Ω DCR max, 0603 | C80165 | https://www.lcsc.com/product-detail/Ferrite-Beads_Murata-Electronics-BLM18PG221SN1D_C80165.html |
| 17 | XAL5030-222MEC | VERIFIED | XAL5030-222MEC | 2.2 µH ±20 %, Isat 9.2 A (30 % drop), Irms 9.7 A, DCR 14.5 mΩ. Plenty of margin for 3 A | C920280 (JLCPCB) | https://jlcpcb.com/partdetail/Coilcraft-XAL5030222MEC/C920280 ; https://www.coilcraft.com/getmedia/49bc46c8-4b2c-45b9-9b6c-2eaa235ea698/xal50xx.pdf |
| 18 | 2.2 µH for TLV62569 at ≤1.5 A | VERIFIED (proposed part) | **Murata DFE252012F-2R2M=P2** | 2.2 µH ±20 %, 2.5x2.0x1.2 mm, Irms 2.3 A, Isat 3.3 A, DCR 82 mΩ max | C576403 | https://www.lcsc.com/product-detail/C576403.html |
| 18b | Sunlord SWPA3015S2R2MT | Exists; specs partly UNVERIFIED | (alternate) | 2.2 µH ±20 %, 3x3x1.5 mm, "2 A" rating. DCR and Isat conflict between sources (78 vs 780 mΩ), so check the datasheet | C43389 | https://jlcpcb.com/partdetail/Sunlord-SWPA3015S2R2MT/C43389 |
| 19a | Epson X1E000021017711 | VERIFIED (availability risk) | — | TSX-3225, 40 MHz, ±10 ppm, **CL 9 pF**, **ESR 40 Ω max**. One distributor lists it "Discontinued". Not found on LCSC | — | https://uk.rs-online.com/web/p/crystal-units/1732302 ; https://www.axel-gl.com/en/asone/d/63-4931-77/ |
| 19b | CTS 402F4001XIAR | VERIFIED (**recommended**) | 402F4001XIAR | 40 MHz, ±10 ppm, **CL 10 pF**, **ESR 60 Ω**. **This is a 2016 package (2.0x1.6), not 3225** | C5508101 | https://www.lcsc.com/product-detail/crystals_cts-402f4001xiar_C5508101.html |
| 20 | ABM8-25.000MHZ-B2-T | VERIFIED | Keep it, or better YXC X322525MOB4SI (see findings) | 25 MHz, **CL 18 pF**, ±20 ppm tolerance, **±50 ppm stability** (-20 to +70 °C), ESR 50 Ω, 3225 | C596899 (JLCPCB). Alt. X322525MOB4SI: C9006 (±10 ppm tol / ±20 ppm stab, CL 12 pF, -40 to 85 °C) | https://jlcpcb.com/partdetail/AbraconLLC-ABM8_25_000MHZ_B2T/C596899 ; https://www.lcsc.com/product-detail/C9006.html |
| 21 | PTS810SJM250SMTRLFS | VERIFIED | PTS810SJM250SMTRLFS | SPST-NO SMT, 4.2x3.2x2.5 mm, 250 gf nominal (snippets vary), 50 mA 16 V, J-lead | C116501 | https://www.lcsc.com/product-detail/C116501.html |
| 22a | Würth 150060GS75000 | VERIFIED | 150060GS75000 | Green 0603, 520-525 nm, VF about 3.2 V at 20 mA | C5252984 | https://www.lcsc.com/product-detail/Light-Emitting-Diodes-LED_Wurth-Elektronik-150060GS75000_C5252984.html |
| 22b | Würth 150060RS75000 | VERIFIED | 150060RS75000 | Red 0603, 625 nm, VF about 2.0 V at 20 mA | C3030991 | https://www.lcsc.com/product-detail/Light-Emitting-Diodes-LED_Wurth-Elektronik-150060RS75000_C3030991.html |
| 23 | SM02B-SRSS-TB(LF)(SN) | VERIFIED | SM02B-SRSS-TB(LF)(SN) | JST SH 1.0 mm, 2-pos, side entry, SMT, 1 A 50 V; mates with SHR-02V-S | C160402 | https://www.lcsc.com/product-detail/C160402.html |
| 24 | Würth 61302421121 | VERIFIED | 61302421121 | WR-PHD 2x12 (24 pos), 2.54 mm, straight THT, unshrouded | not found on LCSC | https://www.newark.com/wurth-elektronik/61302421121/board-to-board-connector-vertical/dp/20X0994 ; https://www.digikey.com/en/products/detail/w%C3%BCrth-elektronik/61302421121/4846869 |
| 25 | TPD4E05U06DQAR | VERIFIED | TPD4E05U06DQAR | 4-ch, 0.5 pF, 5.5 V working, ±12 kV contact, USON-10 2.5x1.0 (KiCad pinout in B-section notes) | C138714 | https://www.lcsc.com/product-detail/C138714.html |
| 26a | CL05B104KO5NNNC | VERIFIED | — | 100 nF 16 V X7R ±10 % 0402 | C1525 | https://www.lcsc.com/product-detail/C1525.html |
| 26b | CL05B103KB5NNNC | VERIFIED | — | 10 nF 50 V X7R 0402 | C15195 | https://www.lcsc.com/product-detail/C15195.html |
| 26c | CL05A105KA5NQNC | VERIFIED | — | 1 µF 25 V X5R ±10 % 0402 | C52923 | https://jlcpcb.com/partdetail/C52923 |
| 26d | CL05A106MQ5NUNC | VERIFIED | — | 10 µF 6.3 V X5R ±20 % 0402. The thickness code "5" means 0.50 mm (the same code is listed as 0.50±0.05 mm on CL05A475MP5NRNC). Expect heavy DC-bias derating at 3.3 V | C15525 | https://www.lcsc.com/product-detail/C15525.html |
| 26e | CL05A475MP5NRNC | VERIFIED | — | 4.7 µF 10 V X5R ±20 % 0402, T = 0.50±0.05 mm | C23733 | https://jlcpcb.com/partdetail/CL05A475MP5NRNC/C23733 |
| 26f | CL05C220JB5NNNC | VERIFIED | — | 22 pF 50 V C0G ±5 % 0402 | C70464 | https://www.lcsc.com/product-detail/Multilayer-Ceramic-Capacitors-MLCC-SMD-SMT_Samsung-Electro-Mechanics-CL05C220JB5NNNC_C70464.html |
| 26g | CL05C120JB5NNNC | VERIFIED (exists) | — | 12 pF 50 V C0G ±5 % 0402 | LCSC C# not found | https://www.mouser.com/ProductDetail/Samsung-Electro-Mechanics/CL05C120JB5NNNC?qs=349EhDEZ59qBxmY25XcLQg%3D%3D |
| 26h | CL05B471KB5NNNC | VERIFIED | — | 470 pF 50 V X7R 0402 | C26412 | https://www.lcsc.com/product-detail/Multilayer-Ceramic-Capacitors-MLCC-SMD-SMT_SAMSUNG_CL05B471KB5NNNC_470pF-471-10-50V_C26412.html |
| 26i | CL05B332KB5NNNC | VERIFIED | — | 3.3 nF 50 V X7R 0402 | C26404 | https://lcsc.com/product-detail/Multilayer-Ceramic-Capacitors-MLCC-SMD-SMT_Samsung-Electro-Mechanics_C26404.html |
| 26j | CL21A226MQQNNNE | VERIFIED | — | 22 µF 6.3 V X5R ±20 % 0805 | C5674 | https://lcsc.com/product-detail/Multilayer-Ceramic-Capacitors-MLCC-SMD-SMT_Samsung-Electro-Mechanics-CL21A226MQQNNNE_C5674.html |
| 26k | CL21A106KAYNNNE | VERIFIED | — | 10 µF 25 V X5R ±10 % 0805, T 1.25 mm | C15850 | https://www.lcsc.com/product-detail/C15850.html |
| 26l | CL21A476MQYNNNE | VERIFIED | — | 47 µF 6.3 V X5R ±20 % 0805 | C16780 | https://jlcpcb.com/partdetail/17464-CL21A476MQYNNNE/C16780 |
| 26m | Yageo RC0402FR-07xxxL series | VERIFIED pattern | — | RC0402 + F(1 %) + R(reel) + 07(7") + value + L. Spot checks: RC0402FR-0710KL = C60490, RC0402FR-0749R9L = C87044, RC0402FR-0712K1L (12.1 k, TME), RC0402FR-071KL = C106235. All listed values are standard E24/E96 values | see left | https://www.lcsc.com/product-detail/C60490.html ; https://www.lcsc.com/product-detail/C87044.html ; https://www.tme.com/us/en-us/details/rc0402fr-0712k1l/smd-resistors/yageo/ |
| 26n | RC0402JR-070RL | VERIFIED | — | 0 Ω jumper 0402, 1 A | C60485 | https://www.lcsc.com/product-detail/Chip-Resistor-Surface-Mount_YAGEO-RC0402JR-070RL_C60485.html |

### Item 3: TPS62130 pin questions
- **FSW:** Low (GND) selects 2.5 MHz, the full frequency. High selects 1.25 MHz, half frequency. The datasheet says the part must **start with FSW = Low** to limit inrush current. For 1.25 MHz, tie FSW to **VOUT or PG**, not to VIN. Source: TI datasheet snippet and TPS62130EVM guide.
- **DEF:** Low = nominal VOUT. High = nominal + 5 %. Source: TI SLVA489 "Voltage Margining Using the TPS62130".
- **VFB = 0.8 V**, with VOUT = 0.8 V × (1 + R1/R2). Keep R2 ≤ 120 kΩ so at least about 5 µA flows in the divider. Source: TI SLYT469 and the datasheet.
- KiCad `Regulator_Switching:TPS62130` pins: 1-3 SW, 4 PG, 5 FB, 6 GND, 7 FSW, 8 DEF, 9 SS/TR, 10-12 VIN, 13 EN, 14 VOS, 15-16 GND, 17 EP (GND). Its footprint is `VQFN-16-1EP_3x3mm_P0.5mm_EP1.68x1.68mm_ThermalVias`.

### Item 5: LAN8720A strap and termination questions
- **nINTSEL (pin 2, LED2/nINTSEL)** is latched at power-up and on the rising edge of nRST. The pin has an internal pull-up, so the default is nINTSEL = 1, which gives nINT mode (REF_CLK In, 50 MHz fed into XTAL1/CLKIN). **Pulling it low with an external pull-down (nINTSEL = 0) selects REF_CLK Out mode.** In that mode a 25 MHz crystal is multiplied to 50 MHz, output on pin 14 nINT/REFCLKO, and the interrupt is not available. The recommended strap is **10 kΩ to GND**. With the pull-down fitted, LED2 becomes **active-high**, so wire LED2 → resistor → LED anode, LED cathode → GND, with the 10 k pull-down in parallel. Sources: LAN8720A datasheet snippets and the Microchip schematic checklist.
- A related strap, not asked about: **LED1/REGOFF (pin 3)** has an internal pull-down. The default (low) keeps the internal 1.2 V regulator ON and makes LED1 active-high. Pulling it high disables the regulator. Source: datasheet section on REGOFF and LED polarity (manualslib copy).
- **RBIAS = 12.1 kΩ, 1 %** to GND. Source: Microchip "LAN8720A QFN Rev D Schematic Checklist".
- **TXP/TXN/RXP/RXN each need a 49.9 Ω 1 % pull-up to a 3.3 V analog rail** (VDDA, derived from +3.3 V). The PHY-side magnetics center taps (TCT and RCT) also go to that rail and are bypassed. The checklist also suggests optional DNP small caps (≤22 pF) from each line to GND. Source: Microchip schematic checklist.
- KiCad `Interface_Ethernet:LAN8720A` uses the footprint `VQFN-24-1EP_4x4mm_P0.5mm_EP2.5x2.5mm_ThermalVias`.

### Item 6: CP2102N questions
- **RSTb:** use a **1 kΩ pull-up to VIO**, or to VDD if VIO is tied to VDD. Source: CP2102N datasheet snippet.
- **Self-powered:** tie VREGIN and VDD together to the external 3.3 V, which bypasses the internal regulator; VIO also goes to 3.3 V. Sense USB VBUS through a divider: **22.1 kΩ from USB VBUS to the VBUS pin, and 47.5 kΩ from the VBUS pin to GND**. At 5 V this gives about 3.4 V. The divider is required because the VBUS pin abs-max is VIO + 2.5 V and VIH is VIO − 0.6 V. Source: SiLabs KB article "Self powered CP2102N, VBUS pin connection, and VBUS voltage divider".
- KiCad `Interface_USB:CP2102N-Axx-xQFN24` uses the footprint `QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm`. Pins: 3 D+, 4 D-, 5 VIO, 6 VDD, 7 VREGIN, 8 VBUS, 9 RSTb, 20 RXD, 21 TXD, 19 RTS, 23 DTR.

### Item 7: USB-HS ESD replacement
- Recommended part: **TPD2EUSB30DRTR**, 0.7 pF typ, 2 lines.
- KiCad symbol: `Power_Protection:TPD2EUSB30`. Pins: **1 = D+, 2 = D-, 3 = GND**. Footprint: `Package_TO_SOT_SMD:Texas_DRT-3` (confirmed to exist).
- Alternative that needs no new part: use two of the four channels of **TPD4E05U06DQAR** (0.5 pF).
- USBLC6-2SC6 remains fine for the **CP2102N USB full-speed port**.

---

## 2. Pinout answers

### B1. RJ45 with magnetics (HanRun HR911105A)
**Recommended MPN: HanRun HR911105A**, LCSC C12074. It is 10/100 with integrated magnetics, 2 LEDs, tab-down and THT.
- KiCad symbol: `Connector:RJ45_Hanrun_HR911105A_Horizontal`. Its datasheet link is the LCSC HR911105A PDF.
- Footprint: `Connector_RJ:RJ45_Hanrun_HR911105A_Horizontal`. The pads are 1-12 plus 2× SH, which matches the symbol.
- I cross-checked the pin functions against two independent libraries: the ODRI master-board Eagle library and the duodyne KiCad symbol. They agree with the official symbol.

| Pin | Function | Notes |
|---|---|---|
| 1 | TD+ | to LAN8720A TXP (pin 21) |
| 2 | TD− | to TXN (pin 20) |
| 3 | RD+ | to RXP (pin 23) |
| 4 | TCT (TX center tap, PHY side) | to 3.3 V analog, bypassed |
| 5 | RCT (RX center tap, PHY side) | to 3.3 V analog, bypassed |
| 6 | RD− | to RXN (pin 22) |
| 7 | NC | — |
| 8 | GND for the Bob-Smith termination (75 Ω network plus 1000 pF, inside the jack) | chassis/GND |
| 9 | Green LED **anode** | official symbol: triangle from 9 to 10 |
| 10 | Green LED **cathode** | |
| 11 | Yellow LED **cathode** | |
| 12 | Yellow LED **anode** | |
| SH | Shield (2 pads) | chassis |

**RJ45-with-magnetics symbols in the official `Connector.kicad_symdir`:**
- RJ45_Abracon_ARJP11A-MASA-B-A-EMU2
- RJ45_Amphenol_RJMG1BD3B8K1ANR
- RJ45_Bel_SI-60062-F
- RJ45_Bel_V895-1001-AW
- RJ45_Halo_HFJ11-x2450E-LxxRL (with LEDs)
- RJ45_Halo_HFJ11-x2450ERL
- RJ45_Halo_HFJ11-x2450HRL
- **RJ45_Hanrun_HR911105A_Horizontal**
- RJ45_JK00177 (Pulse, GbE PoE)
- RJ45_JK0654219 (Pulse, GbE)
- RJ45_Kycon_G7LX-A88S7-BP-GY
- RJ45_Pulse_JXD6-0001NL
- RJ45_RB1-125B8G1A (UDE, GbE)
- RJ45_Wuerth_74980111211
- RJ45_Wuerth_7499010121A
- RJ45_Wuerth_7499010211A
- RJ45_Wuerth_7499151120 (dual)
- Generic symbols also exist: 8P8C*, RJ45, RJ45_LED*, RJ45_Shielded*.

HR911105A is the most suitable because it is LCSC-stocked, 10/100, has 2 LEDs, and its symbol and footprint pair are both official.

### B2. Raspberry Pi 22-pin 0.5 mm camera connector
Source: raspberrypi/documentation, `documentation/asciidoc/accessories/camera/advanced.adoc`, "Camera connector pinout (22-Pin)". It applies to the Pi Zero series, the CM IO boards and Pi 5. The document names the compatible connector as Amphenol F32Q-1A7H1-11022, which is top contact. Signal direction is from the Pi's side. SCL and SDA are pulled up to 3.3 V on the Pi. CAM_IO0 is typically an active-high power enable.

| Pin | Name | Pin | Name |
|---|---|---|---|
| 1 | GND | 12 | CAM_DP2 |
| 2 | CAM_DN0 | 13 | GND |
| 3 | CAM_DP0 | 14 | CAM_DN3 |
| 4 | GND | 15 | CAM_DP3 |
| 5 | CAM_DN1 | 16 | GND |
| 6 | CAM_DP1 | 17 | CAM_IO0 (e.g. power enable, 3.3 V) |
| 7 | GND | 18 | CAM_IO1 (e.g. clock/LED, 3.3 V) |
| 8 | CAM_CN (clock −) | 19 | GND |
| 9 | CAM_CP (clock +) | 20 | SCL (3.3 V) |
| 10 | GND | 21 | SDA (3.3 V) |
| 11 | CAM_DN2 | 22 | 3V3 (supply output) |

**Pi 5 display:**
- Pi 5 has 2 × "mini 22-pin, 0.5 mm pitch, combined CSI (camera)/DSI (display) ports". Source: `documentation/asciidoc/computers/raspberry-pi/introduction.adoc`.
- A Raspberry Pi forum answer (thread t=364068, seen only as a search snippet because the forum is blocked) says the Pi 5 DSI pinout is **the same as the CM4IO CAM1 22-pin connector**. That means the pin positions are identical to the table above, with lanes and clock driven by the Pi in DSI mode.
- I did **not** verify whether the CM4 IO board's separate DISP1 connector uses the same mapping.

For the ESP32-P4 (2-lane CSI), connect D0, D1 and the clock. Leave D2/D3 unconnected or grounded as layout dictates.

### B3. ESP32-P4 JTAG, 32 kHz crystal, GPIO34
**Pad JTAG.** Sources: ESP-IDF v5.5 `components/soc/esp32p4/register/hw_ver3/soc/io_mux_reg.h` (FUNC_GPIO2_MTCK ... FUNC_GPIO5_MTDO = function 0) and `docs/en/api-guides/jtag-debugging/esp32p4.inc`.

| Signal | GPIO |
|---|---|
| MTCK (TCK) | GPIO2 |
| MTDI (TDI) | GPIO3 |
| MTMS (TMS) | GPIO4 |
| MTDO (TDO) | GPIO5 |

- By default JTAG is routed to the built-in USB-Serial-JTAG on GPIO24 (D−) and GPIO25 (D+), not to the pads.
- Pad JTAG needs an eFuse: either DIS_USB_JTAG, or JTAG_SEL_ENABLE plus the GPIO34 strap.

**32 kHz crystal:** **XTAL_32K_N = GPIO0**, **XTAL_32K_P = GPIO1**.
- Source: the ESP-IDF `esp_hal_clock/esp32p4/include/hal/clk_tree_ll.h` comment "XTAL_32K_N (i.e. GPIO0)".
- Source: an ESP32-P4 datasheet search snippet (GPIO0 = XTAL_32K_N, GPIO1 = XTAL_32K_P).

**GPIO34 strap:**
- GPIO34 selects the JTAG signal source, and **only after the JTAG_SEL_ENABLE eFuse is burned**. Low at reset selects pad JTAG on GPIO2-5. High selects USB_SERIAL_JTAG. Without that eFuse the level has no effect.
- Sources: ESP-IDF `configure-other-jtag.rst` with `|jtag-sel-gpio| = GPIO34`; esptool `efuse_defs/esp32p4.yaml` ("strapping gpio34").
- Caveat: esptool `esp32p4_v3.0.yaml` says "gpio25" and the ESP-IDF efuse CSV says "gpio15". These look like doc errors (see esptool issue #1078), and the ESP-IDF docs and datasheet say GPIO34.
- The other straps: GPIO35 low means download boot (it has an internal pull-up), and GPIO36 must be high for download. GPIO35 = 0 with GPIO36 = 0 is invalid. The full strap set is GPIO34-38. Source: esptool boot-mode-selection docs for P4.

### B4. KiCad footprint existence (HTTP 200 from kicad-footprints master)
| Requested | Exists? | Correct name if different |
|---|---|---|
| Package_DFN_QFN:VQFN-16-1EP_3x3mm_P0.5mm_EP1.68x1.68mm_ThermalVias | YES | — |
| Package_TO_SOT_SMD:SOT-23-5 | YES | — |
| Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm | YES (the CP2102N symbol default) | — |
| Package_DFN_QFN:VQFN-24-1EP_4x4mm_P0.5mm_EP2.5x2.5mm_ThermalVias | YES (the LAN8720A symbol default) | — |
| Package_SON:WSON-8-1EP_8x6mm_P1.27mm_EP3.4x4.3mm | YES | — |
| Package_TO_SOT_SMD:SOT-363_SC-70-6 | YES | — |
| Package_TO_SOT_SMD:SOT-23-6 | YES | — |
| Package_TO_SOT_SMD:SOT-23 | YES | — |
| Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal | YES (A1/A4-A9/A12, B1/B4-B9/B12, 4× SH) | — |
| Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5 | YES (pads 1-10 + 4× SH) | — |
| Connector_FFC-FPC:Hirose_FH12-22S-0.5SH_1x22-1MP_P0.50mm_Horizontal | YES | Pi-matching top-contact option: `Connector_FFC-FPC:Amphenol_F32Q-1A7x1-11022_1x22-1MP_P0.5mm_Horizontal` (exists) |
| Connector:FanPinHeader_1x04_P2.54mm_Vertical | YES (descr names Molex 47053-1000) | — |
| Connector_JST:JST_SH_SM02B-SRSS-TB_1x02-1MP_P1.00mm_Horizontal | YES | — |
| Connector_PinHeader_2.54mm:PinHeader_2x12_P2.54mm_Vertical | YES | — |
| Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm | YES | — |
| Crystal:Crystal_SMD_2016-4Pin_2.0x1.6mm | YES (needed if the CTS 402F is used) | — |
| Inductor_SMD:L_Coilcraft_XAL5030-222 | **NO** | **Inductor_SMD:L_Coilcraft_XAL5030-XXX** |
| Diode_SMD:D_SOD-128 | YES | — |
| Fuse:Fuse_1812_4532Metric | YES | — |
| Button_Switch_SMD:SW_SPST_PTS810 | YES (pads 1,1,2,2) | — |
| MountingHole:MountingHole_2.7mm_M2.5_Pad_Via | YES | — |
| LED_SMD:LED_0603_1608Metric | YES | — |
| Resistor_SMD:R_0402_1005Metric | YES | — |
| Capacitor_SMD:C_0402_1005Metric | YES | — |
| Capacitor_SMD:C_0805_2012Metric | YES | — |
| Inductor_SMD:L_0603_1608Metric | YES | — |
| Extra: Package_SON:USON-10_2.5x1.0mm_P0.5mm (TPD4E05U06) | YES | — |
| Extra: Package_TO_SOT_SMD:Texas_DRT-3 (TPD2EUSB30) | YES | — |
| Extra: Connector_RJ:RJ45_Hanrun_HR911105A_Horizontal | YES | — |
| Extra: 2.2 µH small inductor footprints | `L_Sunlord_SWPA3015S` exists. There is **no Murata DFE252012F-specific footprint** (only `L_Murata_DFE201610P`). For the DFE252012F, use `Inductor_SMD:L_1008_2520Metric` after checking it against Murata's land pattern, or make a custom one. | — |

**Other KiCad notes:**
- `Power_Protection:TPD4E05U06DQA` extends TPD4EUSB30. Pins: 1 D1+, 2 D1−, 3 GND, 4 D2+, 5 D2−, 6/7/9/10 NC (flow-through), 8 GND.
- `Connector:Micro_SD_Card_Det_Hirose_DM3AT` extends Micro_SD_Card_Det2. Pins: 1 DAT2, 2 DAT3/CD, 3 CMD, 4 VDD, 5 CLK, 6 VSS, 7 DAT0, 8 DAT1, 9 DET_B, 10 DET_A, SH.
- `Connector:USB_C_Receptacle_USB2.0_16P` matches the USB4105 footprint pad names.

---

## 3. Design-impacting findings
1. **USB-HS ESD:** USBLC6-2SC6 is 3.5 pF max, which exceeds Espressif's ≤1 pF limit for the P4 USB 2.0 HS OTG port. Use **TPD2EUSB30DRTR** (0.7 pF, C97502, KiCad `TPD2EUSB30`, pins 1 D+ / 2 D− / 3 GND) or a TPD4E05U06DQAR on the HS D+/D−. USBLC6 can stay on the CP2102N full-speed port.
2. **PTC fuse MPN is wrong:** MF-MSMF250-2 was not found. Use **MF-MSMF250/16X-2** (2.5 A hold, 16 V, 1812, C210838).
3. **FPC contact side:** FH12-22S-0.5SH(55) is **bottom-contact**, while the Pi's 22-pin connector (Amphenol F32Q-1A7H1-11022) is **top-contact**.
   - With a bottom-contact part, a standard Pi camera cable goes in contacts-down. The physical pin order then reverses: cable conductor 1 lands on connector pin 22.
   - Fix: **either use F32Q-1A7H1-11022** (LCSC C3169253, KiCad footprint `Amphenol_F32Q-1A7x1-11022_1x22-1MP_P0.5mm_Horizontal`) or FH12A-22S-0.5SH(55) (top contact; exact MPN UNVERIFIED) to copy the Pi exactly. Otherwise mirror the netlist (pin n → 23−n) or specify an opposite-side-contact cable.
4. **XAL5030 footprint name:** the correct name is `Inductor_SMD:L_Coilcraft_XAL5030-XXX`.
5. **The TLV62569 input is limited to 2.5-5.5 V.** It must be fed from a ≤5.5 V rail, not directly from a 12 V input. Its VFB is 0.6 V (the TPS62130's is 0.8 V), so compute the dividers separately.
6. **TPS62130 FSW:** GND gives 2.5 MHz. For 1.25 MHz tie FSW to VOUT or PG, never to VIN, because the part must start with FSW low. DEF low gives nominal VOUT.
7. **LAN8720A in REF_CLK-Out mode:**
   - Put a 10 kΩ pull-down on LED2/nINTSEL (pin 2). LED2 then becomes active-high (anode on the pin side, cathode to GND).
   - 50 MHz comes out on pin 14 and goes to an ESP32-P4 REF_CLK input: GPIO32, GPIO44 or GPIO50 per ESP-IDF esp_eth.rst.
   - RBIAS = 12.1 k 1 %. Fit 4 × 49.9 Ω 1 % pull-ups to filtered 3.3 V on TXP/TXN/RXP/RXN, and tie HR911105A pins 4 and 5 (TCT/RCT) to the same rail with bypass.
8. **ESP32-P4 RMII pins overlap straps.** The ESP-IDF P4 RMII options are: TXD0 on GPIO34 or 41, TXD1 on GPIO35 or 42, TX_EN on GPIO33, 40 or 49, CRS_DV on GPIO28, 45 or 51, RXD0 on GPIO29, 46 or 52, RXD1 on GPIO30, 47 or 53.
   - GPIO34 and GPIO35 are strapping pins; GPIO35 low at reset means download mode.
   - If TXD0/TXD1 go on 34/35 (which frees 39-44 for SDMMC), make sure nothing pulls GPIO35 low at reset. I did not verify the LAN8720A input pull configuration on TXD0/TXD1.
9. **25 MHz crystal for the LAN8720A:**
   - ABM8-25.000MHZ-B2-T has ±20 ppm tolerance and ±50 ppm stability. The worst case exceeds the ±50 ppm 100BASE-TX budget (the LAN8720A total-PPM table itself is UNVERIFIED).
   - Its CL is 18 pF, which needs about 27 pF load caps (for example CL05C270JB5NNNC, which exists per RS). 22 pF caps would be under-loaded.
   - Better option: **YXC X322525MOB4SI** (C9006: ±10 ppm / ±20 ppm, CL 12 pF). Use about **18 pF** load caps: CL05C180JB5NNNC, C307443.
10. **40 MHz crystal:**
    - The Epson X1E000021017711 (CL 9 pF, 40 Ω) is flagged discontinued at one distributor and is not on LCSC.
    - The CTS 402F4001XIAR (CL 10 pF, 60 Ω ESR, C5508101) is stocked, but it is a **2016 package**, so use `Crystal_SMD_2016-4Pin_2.0x1.6mm`.
    - The 12 pF C0G load caps (CL05C120JB5NNNC) fit a CL of about 9-10 pF with about 3-4 pF stray.
11. **microSD ESD:** a 4-bit SD bus has 6 lines (CLK, CMD, DAT0-3), so one TPD4E05U06DQAR (4 channels) is not enough. Use two, or a 6-channel TPD6E05U06RVZ (the KiCad symbol exists; I did not verify the MPN's stock).
12. **Flash supply:** ESP32-P4 VDDO_FLASH defaults to 3.3 V (1.8 V only via eFuse), so the 3.3 V W25Q256JV is compatible. Place 0.1 µF + 1 µF at VDD_FLASHIO. Source: Espressif ESP32-P4 schematic checklist.
13. **PMEG4050EP at 2.5 A** drops about 0.38-0.43 V and dissipates about 1 W. Give it copper for heat sinking, or use an ideal-diode or MOSFET OR-ing if efficiency matters.
14. **CL05A106MQ5NUNC** (10 µF 6.3 V 0402) loses much of its capacitance at 3.3 V DC bias. Where bulk capacitance matters, use the 0805 22/47 µF parts.

## Not verifiable here, so marked UNVERIFIED
- USBLC6-2SC6 typical capacitance (only the 3.5 pF max is confirmed).
- LAN8720A crystal "total ppm budget" row.
- The exact MPN FH12A-22S-0.5SH(55).
- SWPA3015S2R2MT DCR and Isat.
- Whether the CM4IO DISP1 connector uses the same pinout as Pi 5.
- The LAN8720A TXD0/TXD1 internal pulls.
- The ESP32-P4 40 MHz crystal ESR limit.
