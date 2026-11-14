# FAULHABER reference documents

These vendor documents are included unchanged so the CANopen implementation,
electrical connection notes, and README page citations can be reviewed without
searching for a matching manual revision.

Copyright and trademarks remain with DR. FRITZ FAULHABER GMBH & CO. KG. The
documents state that specifications are subject to change; check FAULHABER's
official site for a newer revision before applying the information to different
hardware or firmware.

For short, robotics-oriented definitions of CAN, CANopen, COB-IDs, SDOs, PDOs,
SYNC, NMT, CiA 402, and CSP, see the main README's
[CAN and CANopen glossary](../README.md#can-and-canopen-glossary).

## Included files

| Document | Revision | Pages | SHA-256 |
|---|---|---:|---|
| [Communication / Function Manual CANopen BLDC, document 7000.05037](EN_7000_05037_CANopen_Communication_Function_Manual.pdf) | 2nd edition, 2018-11-09 | 113 | `c6f1101e06f494d76e011bca9087a74a66c91c8429e37b850c5a9269bdeac6c6` |
| [2250 ... BX4 IMC RS/CO product datasheet](EN_2250_BX4_IMC_RSCO_DFF.pdf) | 2026-07-28 edition | 2 | `f43e444cf330ccd2853b7248e5c9c0a6cb1572bcaa121ac3685d83bbfedce73b` |

The printed page numbers in the CANopen manual match the PDF page numbers. A
link containing `#page=N` should open at the cited page in PDF viewers that
support page fragments.

## CANopen manual page map

| Pages | Contents used by this repository |
|---:|---|
| [15–18](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=15) | CANopen overview, identifier distribution, and predefined COB-IDs |
| [18–21](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=18) | PDO transmission types, configuration, mapping, and delivered layouts |
| [22–24](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=22) | Expedited SDO reads/writes and SDO aborts |
| [25–26](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=25) | Emergency object format and error codes |
| [27](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=27) | SYNC object and synchronous PDO triggering |
| [28–32](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=28) | NMT commands, boot-up, node guarding, and heartbeat |
| [37–45](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=37) | CiA 402 state machine, Controlword, Statusword, and mode selection |
| [46–49](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=46) | Position factor and conversion between internal and user units |
| [50–54](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=50) | Profile Position mode and position-command handshake |
| [55–62](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=55) | Homing methods, including method 35, and homing status bits |
| [63–65](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=63) | Profile Velocity mode and Target Velocity |
| [77–78](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=77) | Current controller and continuous/peak current limits |
| [94–100](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=94) | Communication-profile object dictionary, including PDO parameters/maps |
| [101–106](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=101) | Manufacturer-specific object dictionary, including current control |
| [107–111](EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=107) | CiA 402 object dictionary entries used for modes, motion, and feedback |

## Motor datasheet page map

| Page | Contents |
|---:|---|
| [1](EN_2250_BX4_IMC_RSCO_DFF.pdf#page=1) | Motor/controller product data |
| [2](EN_2250_BX4_IMC_RSCO_DFF.pdf#page=2) | Dimensions, options, connector, and CAN/power connection assignments |
