# Predictive Maintenance Gateway

**STM32 (FreeRTOS) + CAN + Embedded Linux (Yocto) + AI**

Two STM32 nodes measure the vibration and temperature of two DC motors and send them over a CAN bus to a Linux gateway. The gateway decodes and stores the data, runs a machine-learning model that detects abnormal motor behaviour, and shows measurements and alerts on a dashboard.


## Architecture

```
 [Motor 1 + MPU9250 + DS18B20]        [Motor 2 + MPU9250 + DS18B20]
            │ I2C / 1-Wire                       │
   [STM32 node 1 – FreeRTOS]           [STM32 node 2 – FreeRTOS]
   acquisition → processing → CAN      acquisition → processing → CAN
            │                                    │
            └──────────── CAN bus (500 kbit/s) ──┘
                              │
              [Linux gateway – Ubuntu now, Raspberry Pi + Yocto later]
              SocketCAN → decoder → SQLite → AI model → dashboard
```

## Repository structure

| Folder | Content |
| --- | --- |
| `firmware/` | STM32CubeIDE projects (FreeRTOS, sensors, CAN) |
| `yocto/meta-gateway/` | Custom Yocto layer: recipes for the gateway image |
| `gateway/` | Python gateway service: CAN decoder, storage, inference |
| `ml/` | Datasets, notebooks and model training |
| `dashboard/` | Monitoring dashboard |
| `docs/` | CAN protocol, wiring, notes, test results |

## Hardware

- 2 × STM32 board with CAN
- 2 × CAN transceiver, 2 × 120 Ω termination resistor
- 2 × small DC motor + driver
- 2 × vibration sensor (MPU9250 / MPU6050), 2 × DS18B20 temperature sensor
- Gateway: Ubuntu 24.04 PC with a USB-CAN adapter (Raspberry Pi 4 later)

## CAN protocol

See [`docs/PROTOCOL.md`](docs/PROTOCOL.md).

## Roadmap

- [ ] Week 1 — FreeRTOS tasks, sensor acquisition, first CAN frame received on Linux
- [ ] Week 2 — Two nodes, gateway decoder, SQLite storage, systemd service, dashboard v1
- [ ] Week 3 — Dataset, anomaly-detection model, live alerts on the dashboard
- [ ] Week 4 — Tests, measured results, Yocto image, documentation, demo video

## Results

_Measured results (detection rate, false alerts, detection delay) will be added in week 4._

## Author

Rayen Lamouchi — [GitHub](https://github.com/Rlamouchi) · [LinkedIn](https://www.linkedin.com/in/lamouchi-rayen-24225a400)
