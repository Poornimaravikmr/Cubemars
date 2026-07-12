# Cubemars Motor Control using Teensy 4.1

This repository contains Arduino examples for controlling Cubemars AK Series motors using a Teensy 4.1 over CAN communication with the FlexCAN_T4 library.

## Projects

### Cubemars_PC
Independent position control of two Cubemars motors.

### Sine_spd
Generates sinusoidal position trajectories with velocity and acceleration feedforward.

### Cubemars_PC_t
Time-based position control.

### mcp2551_sine
Generates sinusoidal position trajectories for both motors.


## Hardware

- Teensy 4.1
- MCP2551 CAN Transceiver
- Cubemars AK Series Motors
---

## 🔌 Teensy 4.1 ↔ MCP2551 Connections

| Teensy 4.1 Pin | MCP2551 Pin | Description |
|---------------|-------------|-------------|
| Pin 22 (CAN1_TX) | TXD | CAN transmit |
| Pin 23 (CAN1_RX) | RXD | CAN receive |
| 5V | VCC | Power supply |
| GND | GND | Common ground |

---

## 🔌 MCP2551 ↔ Cubemars Motor Connections

| MCP2551 Pin | Motor Connection |
|-------------|------------------|
| CANH | CANH |
| CANL | CANL |

---

## Software

- Arduino IDE


## Repository Structure

```text
Cubemars/
├── Cubemars_PC/
├── Sine_spd/
├── Cubemars_PC_t/
├── MCP2551_sine/
└── Documentation/
```

## Author

Poornima Ravikumar
