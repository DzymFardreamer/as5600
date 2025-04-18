# AS5600 MicroPython Driver

A simple MicroPython driver for the **AS5600** magnetic rotary encoder.

This library gives you clean access to:
- Reading the angle (0-4095 ticks → 0-360 degrees).
- Configuring power modes, filters, and output types.
- Burning permanent settings into the AS5600 NVM.
- Checking magnet presence and strength easily.

## 🛠️ How to Use

### Quick Example

```python
from machine import Pin, I2C
from as5600 import AS5600, PowerMode, OutputStage
import time

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
sensor = AS5600(i2c)

print("I2C devices:", i2c.scan())

# Read status
print("Magnet Status:", sensor.read_status())

# Read current angle
while True:
    angle = sensor.read_angle()
    print("Angle:", angle)
    time.sleep_ms(100)
```

## 📚 Features

- ✅ Read raw angle and scaled angle.
- ✅ Detect if the magnet is present, too strong or too weak.
- ✅ Read AGC (automatic gain control) and magnet magnitude.
- ✅ Configure:
  - Power modes (Normal, Low Power 1/2/3).
  - Output stages (Analog Full, Analog Reduced, PWM).
  - Slow and Fast filters.
  - PWM base frequency.
  - Watchdog timer.
- ✅ Burn permanent settings (`burn_angle` and `burn_setting`).

## ⚠️ Warnings

- `burn_angle()` can only be done **3 times** max.
- `burn_setting()` can be done **only once** if no previous burns.
- Always check `read_zmco()` before burning settings!

# 📄 License

**GNU General Public License v3.0**

This project is licensed under the GNU GPLv3.

You are free to use, modify, and distribute this code,  
under the conditions of the GNU General Public License Version 3.

Please refer to the full license text at:  
[https://www.gnu.org/licenses/gpl-3.0.html](https://www.gnu.org/licenses/gpl-3.0.html)

---

Originally developed by **Dzym Fardreamer**.  
GitHub: [https://github.com/DzymFardreamer/as5600](https://github.com/DzymFardreamer/as5600)

## ✨ Credits

- AMS AS5600 datasheet for all register mappings.
