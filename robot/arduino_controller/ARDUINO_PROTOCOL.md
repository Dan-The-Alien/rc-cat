# Arduino Serial Communication Protocol

## Overview
The Raspberry Pi communicates with an Arduino Uno over serial to control a servo and ESC. The Arduino receives text commands and manages all PWM output directly.

## Serial Configuration
- **Baud Rate:** 9600 bps (configurable via `ARDUINO_BAUD` environment variable)
- **Data Bits:** 8
- **Stop Bits:** 1
- **Parity:** None
- **Flow Control:** None
- **Line Ending:** `\n` (newline)

## Command Format
All commands are ASCII text strings terminated with a newline character (`\n`).

### Commands

#### 1. Set Steering Servo
```
STEER:<pulse_us>
```
- **Parameter:** `pulse_us` = Pulse width in microseconds (integer)
- **Range:** 1000–2000 µs (1ms–2ms)
- **Neutral:** 1500 µs
- **Example:** `STEER:1500\n`
- **Response:** None (silent success)

#### 2. Set Throttle/ESC
```
THROTTLE:<pulse_us>
```
- **Parameter:** `pulse_us` = Pulse width in microseconds (integer)
- **Range:** 1500–2000 µs (idle to full forward)
- **Idle/Neutral:** 1500 µs
- **Example:** `THROTTLE:1700\n`
- **Response:** None (silent success)

#### 3. Stop (Idle)
```
STOP
```
- **Effect:** Sets steering to neutral and throttle to idle (1500 µs)
- **Example:** `STOP\n`
- **Response:** None (silent success)

## Error Handling
- Invalid commands should be ignored (optional: log warning to serial monitor)
- Out-of-range values should be clamped to valid range
- Malformed commands should be safely rejected

## Example Communication Sequence

```
Raspberry Pi → Arduino:  STEER:1500
Raspberry Pi → Arduino:  THROTTLE:1500
Raspberry Pi → Arduino:  STEER:1600
Raspberry Pi → Arduino:  THROTTLE:1450
Raspberry Pi → Arduino:  STOP
```

## Hardware Configuration

### Arduino Pins
- **Servo PWM Output:** Pin 9 (or configurable)
- **ESC PWM Output:** Pin 10 (or configurable)
- **Serial RX:** Pin 0 (hardware UART)
- **Serial TX:** Pin 1 (hardware UART)

### Servo/ESC Typical Requirements
- **PWM Frequency:** 50 Hz
- **Pulse Width:** 1000–2000 µs (typical)
- **Throttle Range:** 1500–2000 µs for forward-only control
- **Neutral Position:** 1500 µs

## Environment Variables (Raspberry Pi)

Configure these on the Raspberry Pi to match your hardware:

```bash
export ARDUINO_PORT="/dev/ttyUSB0"    # Serial port (default)
export ARDUINO_BAUD="9600"            # Baud rate (default)
```

On Windows, use `COM3` or appropriate COM port instead of `/dev/ttyUSB0`.

## Troubleshooting

1. **No response from Arduino:**
   - Check serial port: `ls /dev/tty*` on Linux
   - Verify baud rate matches
   - Ensure Arduino sketch is running

2. **Servo/ESC not moving:**
   - Verify GPIO pins in Arduino sketch
   - Check power supply to Arduino
   - Confirm PWM frequency is 50 Hz

3. **Garbled output:**
   - Verify baud rate matches (9600 default)
   - Check USB cable quality
   - Try different USB port
