# MotorDriver - Pololu 25D Gearmotor + Dual MC33926 Driver Control

Arduino Mega 2560 control system for a Pololu 25D gearmotor using a Dual MC33926 motor driver with encoder feedback and current monitoring.

## Overview

This project provides a complete control system for driving and monitoring a Pololu 25D gearmotor using an Arduino Mega 2560 microcontroller. The system features:

- **Quadrature encoder feedback** for position and velocity estimation
- **Proportional (P) control** for position regulation
- **Current sensing** via the MC33926 motor driver
- **Two control modes**: Position reference (pos_ref) or Velocity reference (vel_ref)
- **50 Hz control loop** with deterministic timing

## Hardware Components

- **Microcontroller**: Arduino Mega 2560
- **Motor Driver**: Dual MC33926 Motor Driver
- **Motor**: Pololu 25D Gearmotor (48 counts/revolution encoder)
- **Encoder**: Quadrature encoder on motor (48 CPR)

## Pin Configuration

### Motor Control Pins (motor.cpp)

Modify the pin definitions in `src/motor.cpp` to match your Arduino connections:

```cpp
const uint8_t EN_PIN  = 7;      // Driver enable pin
const uint8_t IN1_PIN = 8;      // Motor direction control 1
const uint8_t IN2_PIN = 9;      // Motor direction control 2
const uint8_t D1_PIN  = 11;     // Driver disable pin 1 (D1 HIGH = disabled)
const uint8_t D2_PIN  = 12;     // Driver disable pin 2 (D2 LOW = disabled)
const uint8_t FB_PIN  = A0;     // Current feedback analog pin (525 mV/A)
```

**Pin Connections:**
- **EN_PIN (7)**: Motor driver enable pin
- **IN1_PIN (8)**: Motor forward control (PWM output)
- **IN2_PIN (9)**: Motor reverse control (PWM output)
- **D1_PIN (11)**: Disable pin 1 (active-high disable)
- **D2_PIN (12)**: Disable pin 2 (active-low disable)
- **FB_PIN (A0)**: Current feedback (analog input)

### Encoder Pins (encoder.cpp)

```cpp
const uint8_t ENC_A_PIN = 2;   // Encoder channel A (interrupt-capable)
const uint8_t ENC_B_PIN = 3;   // Encoder channel B (interrupt-capable)
```

**Pin Connections:**
- **ENC_A_PIN (2)**: Encoder phase A (must use interrupt-capable pin)
- **ENC_B_PIN (3)**: Encoder phase B (must use interrupt-capable pin)

> **Note**: Pins 2 and 3 on Arduino Mega 2560 support external interrupts. Use these pins or other interrupt-capable pins (18-21) as needed.

## Control Modes

This project supports two alternative control approaches:

### Mode 1: Position Reference (pos_ref) - Fixed Target

**File**: `sketches/main_posRef_currentFB.cpp`

Use this mode when you want the motor to reach a fixed position setpoint:

- **Configuration**:
  - `SETPOINT_REV`: Target position in revolutions
  - `CURRENT_LIMIT`: Maximum allowable current
  - `KP`: Proportional gain (tune for response speed)
  - `MAX_RUNTIME_MS`: Maximum test duration

- **Operation**:
  1. Motor runs until position setpoint is reached (within `POS_TOL`)
  2. Control stops when position tolerance is met
  3. Control also stops if current exceeds `CURRENT_LIMIT` or timeout is reached

### Mode 2: Velocity Reference (vel_ref) - Serial Input

**File**: `sketches/main_velRef_currentFB.cpp`

Use this mode for real-time control via serial commands:

- **Configuration**:
  - `KP`: Proportional gain for position error correction
  - `CURRENT_LIMIT`: Maximum allowable current
  - `MAX_RUNTIME_MS`: Maximum test duration

- **Operation**:
  1. Motor responds to velocity commands sent via serial
  2. Accepts floating-point velocity values (revolutions/second)
  3. Integrates velocity reference to generate position setpoint
  4. Maintains position using proportional control

## Running the Code

### Setup Instructions

1. **Install PlatformIO** (recommended) or Arduino IDE
2. **Connect your Arduino Mega 2560** to your computer
3. **Verify pin connections** match your hardware setup
4. **Select control mode** (see below)

### Building and Uploading

#### Using PlatformIO (Recommended)

```bash
# Build the project
platformio run

# Upload to Arduino Mega 2560
platformio run --target upload

# Monitor serial output
platformio device monitor --baud 115200
```

#### Using Arduino IDE

1. Open `src/main.cpp` in Arduino IDE
2. Select Board: **Arduino Mega 2560**
3. Select appropriate COM port
4. Click **Upload**
5. Open Serial Monitor (115200 baud)

### Switching Control Modes

To switch between pos_ref and vel_ref modes:

1. **Copy the desired control sketch** into `src/main.cpp`:
   - For **position control**: Copy contents of `sketches/main_posRef_currentFB.cpp`
   - For **velocity control**: Copy contents of `sketches/main_velRef_currentFB.cpp`

2. **Configure parameters** in the copied code:
   ```cpp
   // Example for pos_ref mode
   const float SETPOINT_REV = -35.0f;  // Change target position
   const float CURRENT_LIMIT = 1.4f;   // Change current limit
   const float KP = 25.0f;             // Tune proportional gain
   ```

3. **Build and upload** the modified code

## Serial Communication

### Baud Rate
```
115200 baud
```

### CSV Output Format

Both modes output data in CSV format for easy analysis:

**pos_ref mode:**
```
time_ms,position_rev,error_rev,pwm,current_A,maxCurrent_A
```

**vel_ref mode:**
```
time_ms,velocity_ref_rev_s,velocity_rev_s,position_ref_rev,position_rev,encoder_count,error_rev,pwm,speed_rpm,current_A,maxCurrent_A,loop_period_us,loop_exe_us
```

### pos_ref Mode - User Input

The system waits for a serial command to start:
```
Send: s
Press Enter/Return
```

Motor will enable in 3 seconds and begin tracking the setpoint.

### vel_ref Mode - Velocity Commands

Send floating-point velocity values via serial (in revolutions/second):
```
Send: 0.5
Send: -0.3
Send: 0
```

Press Enter after each value to apply the command.

## Configuration Parameters

### Control Loop
```cpp
const float TS = 0.020f;  // Sample time: 20 ms (50 Hz)
const uint32_t MAX_RUNTIME_MS = 15000;  // Test duration
```

### Position Control (pos_ref)
```cpp
const float SETPOINT_REV = -100.0f;  // Target position [revolutions]
const float POS_TOL = 1.00f;          // Tolerance for reaching setpoint
const float KP = 20.0f;               // Proportional gain
const float CURRENT_LIMIT = 3.0f;     // Max current cutoff [Amperes]
```

### Velocity Control (vel_ref)
```cpp
const float KP = 10.0f;               // Proportional gain
const float CURRENT_LIMIT = 1.4f;     // Max current cutoff [Amperes]
```

### Motor Control
```cpp
const int PWM_MIN = 30;   // Minimum PWM to overcome static friction
const int PWM_MAX = 130;  // Maximum PWM output (0-255 range)
```

## File Structure

```
MotorDriver/
├── src/
│   ├── main.cpp              # Main control loop (currently pos_ref mode)
│   ├── motor.h/motor.cpp     # Motor driver functions
│   ├── encoder.h/encoder.cpp # Quadrature encoder decoder
│   ├── controller.h/controller.cpp  # Proportional controller
├── sketches/
│   ├── main_posRef_currentFB.cpp    # Position reference mode template
│   ├── main_velRef_currentFB.cpp    # Velocity reference mode template
├── platformio.ini            # PlatformIO configuration
└── README.md                 # This file
```

## Module Descriptions

### motor.cpp
- **motorInit()**: Initialize motor control pins
- **driverEnable()**: Enable the MC33926 driver
- **shutdown()**: Disable the MC33926 driver
- **motorForward(pwm)**: Drive motor forward with PWM (0-255)
- **motorReverse(pwm)**: Drive motor reverse with PWM (0-255)
- **motorStop()**: Stop motor (no PWM)
- **getMotorCurrent()**: Read current feedback (Amperes)

### encoder.cpp
- **encoderInit(ts)**: Initialize encoder with sample time
- **encoderUpdate()**: Update position/velocity estimates
- **getMotorRevs()**: Get current position (revolutions)
- **getMotorSpeedRevs()**: Get velocity (revolutions/second)
- **getMotorSpeedRPM()**: Get speed (RPM)
- **resetEncoderCount()**: Reset encoder to zero position

### controller.cpp
- **propCTRL(KP, setpoint, measurement)**: Proportional controller
  - Returns PWM command (-255 to 255)
  - Includes dead-band and saturation handling

## Tuning Guide

### Proportional Gain (KP)

Adjust `KP` to control response speed:

- **Too low** (e.g., 5-10):
  - Motor responds slowly
  - Takes longer to reach setpoint
  - More stable but sluggish

- **Good range** (e.g., 15-30):
  - Balanced response
  - Reaches setpoint smoothly
  - Minimal oscillation

- **Too high** (e.g., 50+):
  - Motor oscillates around setpoint
  - Overshoot and instability
  - May exceed current limits

**Tuning process**:
1. Start with low KP (10)
2. Increase gradually while monitoring current
3. Find the fastest response without oscillation
4. Stay below current limit

### Current Limit

The current limit provides hardware protection:

- **Too low**: Motor may not move (insufficient torque)
- **Typical values**: 1.0-3.0 Amperes depending on load
- **Too high**: Risk of motor/driver damage

Check your motor specifications for safe continuous current ratings.

## Data Analysis

Both modes output CSV format suitable for analysis in:
- Excel/Google Sheets
- MATLAB
- Python (pandas, matplotlib)

Example Python script:
```python
import pandas as pd

# Load data
data = pd.read_csv('motor_test.csv')

# Plot position
import matplotlib.pyplot as plt
plt.plot(data['time_ms'], data['position_rev'])
plt.xlabel('Time (ms)')
plt.ylabel('Position (revolutions)')
plt.grid()
plt.show()
```

## Troubleshooting

### Motor doesn't move
- Check pin connections match pin definitions in `motor.cpp`
- Verify `SETPOINT_REV` has sufficient magnitude
- Check `KP` is not too low
- Verify motor power supply is connected

### Excessive current draw
- Reduce `KP` (slower response)
- Reduce target position `SETPOINT_REV` magnitude
- Check for mechanical friction or load issues
- Verify `CURRENT_LIMIT` isn't set too high

### Encoder not reading
- Verify ENC_A_PIN and ENC_B_PIN are interrupt-capable pins (2-3 or 18-21 on Mega)
- Check encoder wiring (A/B phase signals)
- Monitor serial output for encoder count changes

### Inconsistent timing
- vel_ref mode prints loop timing information
- Check `loop_period_us` stays near 20000 (20 ms)
- If timing varies, reduce other computation in loop

## References

- [Arduino Mega 2560 Pinout](https://www.arduino.cc/en/uploads/Main/Arduino_Mega_2560_R3_Pinout_Diagram.jpg)
- [Pololu MC33926 Motor Driver](https://www.pololu.com/product/713)
- [Pololu 25D Gearmotor](https://www.pololu.com/product/1627)
- [PlatformIO Documentation](https://docs.platformio.org/)

## License

This project is provided as-is for educational and research purposes.

## Support

For issues or questions:
1. Check the Troubleshooting section above
2. Verify all pin connections
3. Review serial output CSV data for error patterns
4. Consult motor driver and encoder datasheets
