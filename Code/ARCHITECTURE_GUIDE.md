# Freenove Robot Dog - Architecture & Hardware Communication Guide

## Overview

This repository contains code for controlling a Freenove robot dog on Raspberry Pi 5. The system uses a **client-server architecture** where:
- **Server** (runs on Raspberry Pi): Handles all hardware communication
- **Client** (can run on any computer): Provides GUI interface and sends commands via TCP/IP

---

## What `setup.py` Does

The `setup.py` script sets up the Raspberry Pi environment with all required dependencies:

1. **Creates Python3 symlink** (line 4): Links `python` → `python3` in `/usr/bin`
2. **Updates package list** (lines 6-9): Runs `apt-get update` (up to 3 retries)
3. **Installs rpi-ws281x-python** (lines 10-13): LED control library (WS2812/NeoPixel LEDs)
4. **Installs mpu6050 library** (lines 14-17): IMU sensor library for balance/attitude
5. **Installs PyQt5 dependencies** (lines 18-21): GUI framework for client interface
6. **Verification** (lines 22-26): Uses bit flags (0x01, 0x02, 0x04, 0x08) to track success; only reports success if all 4 steps complete (0x0F = 15)

**Usage**: Run `sudo python3 setup.py` on the Raspberry Pi before first use.

---

## Architecture Overview

### Directory Structure

```
Code/
├── Server/          # Runs on Raspberry Pi - hardware control
│   ├── main.py      # Server entry point with PyQt5 GUI
│   ├── Server.py    # TCP server for client communication
│   ├── Servo.py     # Servo motor control
│   ├── PCA9685.py   # PWM driver chip (I2C)
│   ├── IMU.py       # Inertial Measurement Unit (MPU6050)
│   ├── Led.py       # LED strip control
│   ├── Ultrasonic.py # Distance sensor
│   ├── ADS7830.py   # ADC for battery voltage
│   ├── Buzzer.py    # Buzzer control
│   └── Control.py   # Main robot control logic
│
├── Client/          # Runs on any computer - GUI interface
│   ├── Main.py      # Client GUI entry point
│   ├── Client.py    # TCP client for server communication
│   └── ...
│
└── Libs/            # External libraries
    ├── mpu6050/     # IMU sensor library
    └── rpi-ws281x-python/  # LED control library
```

---

## Hardware Communication Patterns

The system uses several communication protocols:

### 1. **I2C Communication** (Two-Wire Interface)

Used for:
- **PCA9685** (0x40): 16-channel PWM driver for servos
- **ADS7830** (0x48): ADC for battery voltage monitoring
- **MPU6050** (0x68): IMU sensor

**Example - PCA9685 (PWM Driver)**:
```python
# From PCA9685.py
import smbus

class PCA9685:
    def __init__(self, address=0x40):
        self.bus = smbus.SMBus(1)  # I2C bus 1
        self.address = address
    
    def write(self, reg, value):
        """Write 8-bit value to I2C register"""
        self.bus.write_byte_data(self.address, reg, value)
    
    def setPWM(self, channel, on, off):
        """Set PWM pulse width"""
        self.write(LED0_ON_L + 4*channel, on & 0xFF)
        self.write(LED0_ON_H + 4*channel, on >> 8)
        self.write(LED0_OFF_L + 4*channel, off & 0xFF)
        self.write(LED0_OFF_H + 4*channel, off >> 8)
```

**Example - Servo Control**:
```python
# From Servo.py
from PCA9685 import PCA9685

class Servo:
    def __init__(self):
        self.pwm = PCA9685(address=0x40)
        self.pwm.setPWMFreq(50)  # 50Hz for servos
    
    def setServoAngle(self, channel, angle):
        # Convert angle (0-180°) to PWM value (102-512)
        pwm_value = self.map(angle, 0, 180, 102, 512)
        self.pwm.setPWM(channel, 0, int(pwm_value))
```

### 2. **GPIO Communication**

Used for:
- **Ultrasonic sensor**: GPIO pins 27 (trigger), 22 (echo)
- **Buzzer**: GPIO pin 17

**Example - Ultrasonic Sensor**:
```python
# From Ultrasonic.py
from gpiozero import DistanceSensor

class Ultrasonic:
    def __init__(self):
        self.sensor = DistanceSensor(
            echo=22, 
            trigger=27, 
            max_distance=3  # 3 meters
        )
    
    def get_distance(self):
        return int(self.sensor.distance * 100)  # Convert to cm
```

**Example - Buzzer**:
```python
# From Buzzer.py
from gpiozero import Buzzer

buzzer = Buzzer(17)  # GPIO pin 17

class Buzzer:
    def run(self, command):
        if command != "0":
            buzzer.on()
        else:
            buzzer.off()
```

### 3. **SPI Communication**

Used for:
- **LED strips** (on PCB v2 with Raspberry Pi 5): SPI interface

**Example** (see `Server/rpi_ledpixel.py` and `Server/spi_ledpixel.py`)

### 4. **Camera Communication**

Used for:
- **Raspberry Pi Camera**: Video streaming

**Example - Camera**:
```python
# From Server.py
from picamera2 import Picamera2

camera = Picamera2()
camera.configure(camera.create_video_configuration(main={"size": (400, 300)}))
```

---

## How to Write Your Own Hardware Communication Code

### Step 1: Identify the Hardware Interface

Determine which protocol your hardware uses:
- **I2C**: Sensors, PWM drivers, ADCs (uses `smbus`)
- **GPIO**: Simple digital I/O (uses `gpiozero` or `RPi.GPIO`)
- **SPI**: High-speed serial (LED strips, displays)
- **UART/Serial**: Serial communication (GPS, some sensors)

### Step 2: Create a Hardware Module

Create a new Python file in `Server/` directory (e.g., `MySensor.py`):

```python
# Server/MySensor.py
import smbus  # For I2C
# OR
from gpiozero import DigitalInputDevice  # For GPIO
# OR
import spidev  # For SPI

class MySensor:
    def __init__(self):
        # Initialize communication
        # I2C example:
        self.bus = smbus.SMBus(1)
        self.address = 0x50  # Your device address
        
        # GPIO example:
        # self.pin = DigitalInputDevice(18)
    
    def read_data(self):
        # Read from hardware
        # I2C example:
        data = self.bus.read_byte_data(self.address, 0x00)
        return data
        
        # GPIO example:
        # return self.pin.value
    
    def write_data(self, value):
        # Write to hardware (if needed)
        self.bus.write_byte_data(self.address, 0x00, value)
```

### Step 3: Integrate with Server

Add your hardware to `Server.py`:

```python
# In Server/Server.py
from MySensor import MySensor

class Server:
    def __init__(self):
        # ... existing code ...
        self.my_sensor = MySensor()  # Add your sensor
    
    def receive_instruction(self):
        # ... existing code ...
        elif cmd.CMD_MY_SENSOR in data:
            value = self.my_sensor.read_data()
            command = cmd.CMD_MY_SENSOR + '#' + str(value) + '\n'
            self.send_data(self.connection1, command)
```

### Step 4: Add Command Definition

Add your command to `Server/Command.py`:

```python
# In Server/Command.py
class COMMAND:
    # ... existing commands ...
    CMD_MY_SENSOR = "CMD_MY_SENSOR"
```

### Step 5: (Optional) Add Client Interface

If you want GUI control, add to `Client/Main.py`:

```python
# In Client/Main.py (MyWindow class)
def my_sensor_action(self):
    command = cmd.CMD_MY_SENSOR + '\n'
    self.client.send_data(command)
```

---

## Common Hardware Communication Patterns

### Pattern 1: I2C Sensor Reading

```python
import smbus

class I2CSensor:
    def __init__(self, address=0x50):
        self.bus = smbus.SMBus(1)
        self.address = address
    
    def read_register(self, register):
        """Read 8-bit register"""
        return self.bus.read_byte_data(self.address, register)
    
    def write_register(self, register, value):
        """Write 8-bit register"""
        self.bus.write_byte_data(self.address, register, value)
    
    def read_word(self, register):
        """Read 16-bit word (2 bytes)"""
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        return (high << 8) | low
```

### Pattern 2: GPIO Digital I/O

```python
from gpiozero import DigitalInputDevice, DigitalOutputDevice

class GPIODevice:
    def __init__(self, input_pin=18, output_pin=19):
        self.input = DigitalInputDevice(input_pin)
        self.output = DigitalOutputDevice(output_pin)
    
    def read(self):
        return self.input.value
    
    def write(self, value):
        self.output.value = value
```

### Pattern 3: PWM Output (GPIO)

```python
from gpiozero import PWMOutputDevice

class PWMMotor:
    def __init__(self, pin=12):
        self.pwm = PWMOutputDevice(pin)
    
    def set_speed(self, speed):  # speed: 0.0 to 1.0
        self.pwm.value = speed
```

### Pattern 4: SPI Communication

```python
import spidev

class SPIDevice:
    def __init__(self, bus=0, device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 1000000  # 1 MHz
    
    def transfer(self, data):
        """Send and receive data"""
        return self.spi.xfer2(data)
```

---

## Communication Flow

```
Client (Main.py)
    ↓ TCP/IP (commands as strings)
Server (Server.py)
    ↓ Direct hardware access
Hardware Modules (Servo.py, IMU.py, etc.)
    ↓ I2C/GPIO/SPI
Physical Hardware
```

**Command Format**: `COMMAND#PARAM1#PARAM2\n`

Example: `CMD_MOVE_FORWARD#8\n` (move forward at speed 8)

---

## Key Libraries Used

1. **smbus**: I2C communication
2. **gpiozero**: GPIO control (recommended, user-friendly)
3. **RPi.GPIO**: Alternative GPIO library (lower-level)
4. **picamera2**: Raspberry Pi camera
5. **PyQt5**: GUI framework
6. **numpy**: Numerical operations (IMU, control)
7. **spidev**: SPI communication (LEDs)

---

## Tips for Hardware Development

1. **Always check I2C addresses**: Use `i2cdetect -y 1` to find device addresses
2. **Use pull-up resistors**: I2C requires pull-up resistors (often built into sensors)
3. **Check GPIO pin conflicts**: Make sure pins aren't used by other hardware
4. **Test incrementally**: Test each hardware module independently before integration
5. **Use threading for blocking operations**: Hardware reads can block, use threads (see `Server/Thread.py`)
6. **Error handling**: Always wrap hardware access in try/except blocks
7. **Voltage levels**: Ensure 3.3V/5V compatibility between Pi and hardware

---

## Example: Adding a New I2C Temperature Sensor

```python
# Server/TemperatureSensor.py
import smbus

class TemperatureSensor:
    def __init__(self, address=0x48):
        self.bus = smbus.SMBus(1)
        self.address = address
    
    def read_temperature(self):
        # Read temperature register (example)
        data = self.bus.read_word_data(self.address, 0x00)
        # Convert to temperature (depends on sensor)
        temp = (data / 32.0) - 256.0
        return round(temp, 2)
```

Then integrate into `Server.py` as shown in Step 3 above.
