# Cubemars Motor Control using Teensy 4.1

This repository contains Arduino examples for controlling Cubemars AK Series motors using a Teensy 4.1 over CAN communication with the FlexCAN_T4 library.

## Projects

### Cubemars_PC
Independent position control of two Cubemars motors.

### Cubemars_PC_Time
Time-based position control.

### Sinusoidal_Position_Control
Generates sinusoidal position trajectories for both motors.

### Sinusoidal_Position_Velocity_Control
Generates sinusoidal position trajectories with velocity and acceleration feedforward.

## Hardware

- Teensy 4.1
- MCP2551 CAN Transceiver
- Cubemars AK Series Motors

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
