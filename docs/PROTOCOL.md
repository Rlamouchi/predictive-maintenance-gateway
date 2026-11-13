# CAN protocol – v0.1

| Parameter | Value |
| --- | --- |
| Bit rate | 500 kbit/s |
| Identifiers | Standard 11-bit |
| Byte order | Little-endian |
| Node 1 (motor 1) | IDs `0x10x` |
| Node 2 (motor 2) | IDs `0x20x` |

Rule: `ID = 0x100 × node_id + message_type`.

## Messages sent by each node

| ID node 1 | ID node 2 | Message | Period | DLC |
| --- | --- | --- | --- | --- |
| `0x101` | `0x201` | Vibration | 100 ms | 5 |
| `0x102` | `0x202` | Temperature | 1 s | 3 |
| `0x103` | `0x203` | Heartbeat / diagnostics | 1 s | 6 |

### Vibration (`0x101` / `0x201`)

| Byte | Field | Type | Unit |
| --- | --- | --- | --- |
| 0–1 | Vibration RMS | uint16 | mg (milli-g) |
| 2–3 | Vibration peak | uint16 | mg |
| 4 | Message counter | uint8 | wraps 255 → 0 |

### Temperature (`0x102` / `0x202`)

| Byte | Field | Type | Unit |
| --- | --- | --- | --- |
| 0–1 | Motor temperature | int16 | 0.01 °C (4250 = 42.50 °C) |
| 2 | Sensor status | uint8 | 0 = OK, 1 = not found, 2 = read error |

### Heartbeat / diagnostics (`0x103` / `0x203`)

| Byte | Field | Type | Notes |
| --- | --- | --- | --- |
| 0 | Node state | uint8 | 0 = init, 1 = running, 2 = error |
| 1 | Error flags | uint8 | bit 0 IMU error, bit 1 temp error, bit 2 queue full, bit 3 CAN TX error |
| 2–5 | Uptime | uint32 | seconds |

## Reserved for later

| ID | Direction | Message |
| --- | --- | --- |
| `0x301` / `0x302` | Gateway → node | Commands (start / stop motor) |

## Changelog

- v0.1 — first version (day 1)
