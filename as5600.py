# ─────────────────────────────────────────────────────────────────────────────
#                     GNU GENERAL PUBLIC LICENSE
#                        Version 3, 29 June 2007
#
# This file is part of the AS5600 MicroPython Driver.
#
# The AS5600 MicroPython Driver is free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This driver is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this driver.  If not, see <https://www.gnu.org/licenses/>.
#
# Author: Dzym Fardreamer
# Repository: https://github.com/DzymFardreamer/as5600
# ─────────────────────────────────────────────────────────────────────────────
from machine import I2C

class PowerMode:
    NOM = 0
    LPM1 = 1
    LPM2 = 2
    LPM3 = 3

class Hysteresis:
    OFF = 0
    LSB1 = 1
    LSB2 = 2
    LSB3 = 3

class OutputStage:
    ANALOG_FULL = 0
    ANALOG_REDUCED = 1
    PWM = 2

class PWMFreq:
    FREQ_115 = 0
    FREQ_230 = 1
    FREQ_460 = 2
    FREQ_920 = 3

class SlowFilter:
    X16 = 0
    X8 = 1
    X4 = 2
    X2 = 3

class FastFilterThreshold:
    SLOW_ONLY = 0
    LSB6 = 1
    LSB7 = 2
    LSB9 = 3
    LSB18 = 4
    LSB21 = 5
    LSB24 = 6
    LSB10 = 7

class AS5600:
    """Tiny driver for the **AS5600** magnetic rotary encoder.

    It wraps the I²C register map with easy methods + enums so you can read
    angles and tweak settings without bit‑twiddling.
    All angles / positions are **12‑bit (0‑4095)** unless noted.
    """

    # ── register addresses (datasheet section 8) ───────────────────────────
    REG_ZMCO          = 0x00  # [RO] burn counter (bits 1:0)
    REG_ZPOS_MSB      = 0x01  # +0x02 = ZPOS LSB  (RW, NVM‑burnable)
    REG_MPOS_MSB      = 0x03  # +0x04 = MPOS LSB  (RW, NVM‑burnable)
    REG_MANG_MSB      = 0x05  # +0x06 = MANG LSB  (RW, NVM‑burnable once)
    REG_CONF_MSB      = 0x07  # +0x08 = CONF LSB  (RW, NVM‑burnable once)
    REG_STATUS        = 0x0B  # [RO]
    REG_RAW_ANGLE_MSB = 0x0C  # +0x0D = RAW ANGLE (RO)
    REG_ANGLE_MSB     = 0x0E  # +0x0F = SCALED ANGLE (RO)
    REG_AGC           = 0x1A  # [RO] automatic gain 0‑255 (0‑128 @3V3)
    REG_MAG_MSB       = 0x1B  # +0x1C = magnitude (RO)
    REG_BURN          = 0xFF  # [WO] NVM burn commands

    # values to poke into REG_BURN
    _BURN_ANGLE   = 0x80  # burns ZPOS + MPOS (max 3 times)
    _BURN_SETTING = 0x40  # burns MANG + CONF (once, only if ZMCO==0)

    # ── bit masks inside CONF (see fig.22) ─────────────────────────────────
    _PM_MASK, _HYST_MASK, _OUTS_MASK  = 0x0003, 0x000C, 0x0030
    _PWMF_MASK, _SF_MASK, _FTH_MASK   = 0x00C0, 0x0300, 0x1C00
    _WD_MASK                          = 0x2000

    # ── bit masks inside STATUS ────────────────────────────────────────────
    _MD = 0x20  # Magnet detected
    _ML = 0x10  # Magnet too weak (Low)
    _MH = 0x08  # Magnet too strong (High)

    # ---------------------------------------------------------------------
    def __init__(self, i2c: I2C, addr: int = 0x36):
        """Create an **AS5600** object.

        Args:
            i2c (machine.I2C): already‑setup I2C bus
            addr (int, optional): 7‑bit device address (default **0x36**)
        """
        self.i2c, self.addr = i2c, addr

    # ── private low‑level helpers ─────────────────────────────────────────
    def _read_word(self, reg: int) -> int:
        """Return 16‑bit big‑endian word from *reg*/*reg+1*."""
        hi, lo = self.i2c.readfrom_mem(self.addr, reg, 2)
        return (hi << 8) | lo

    def _write_word(self, reg: int, value: int):
        """Write 16‑bit big‑endian *value* into *reg*/*reg+1*."""
        self.i2c.writeto_mem(self.addr, reg, bytes([(value >> 8) & 0xFF]))
        self.i2c.writeto_mem(self.addr, reg + 1, bytes([value & 0xFF]))

    def _read_12(self, reg: int) -> int:
        """Read 12‑bit field that starts at *reg* (MSB)."""
        return self._read_word(reg) & 0x0FFF

    def _write_12(self, reg: int, value: int):
        """Write 12‑bit *value* to *reg* (keeps factory upper nibble)."""
        value &= 0x0FFF
        self._write_word(reg, (self._read_word(reg) & 0xF000) | value)

    def _read_byte(self, reg: int) -> int:
        """Read single byte from *reg*."""
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def _write_byte(self, reg: int, value: int):
        """Write single byte to *reg*."""
        self.i2c.writeto_mem(self.addr, reg, bytes([value & 0xFF]))

    # ── CONF field helpers -------------------------------------------------
    def _conf_bits(self, mask: int, shift: int) -> int:
        return (self._read_word(self.REG_CONF_MSB) & mask) >> shift

    def _set_conf_bits(self, mask: int, shift: int, value: int):
        cfg = self._read_word(self.REG_CONF_MSB)
        self._write_word(self.REG_CONF_MSB, (cfg & ~mask) | ((value << shift) & mask))

    # ── burn counter -------------------------------------------------------
    def read_zmco(self) -> int:
        """How many times **ZPOS/MPOS** were burned (0‑3)."""
        return self._read_byte(self.REG_ZMCO) & 0x03

    # ── position registers (12‑bit) ---------------------------------------
    def read_zpos(self) -> int:
        """Get **ZPOS** (zero position) [0‑4095]."""
        return self._read_12(self.REG_ZPOS_MSB)

    def write_zpos(self, val: int):
        """Set **ZPOS**. Effective immediately, burn with :py:meth:`burn_angle` to
        make it permanent.
        """
        self._write_12(self.REG_ZPOS_MSB, val)

    def read_mpos(self) -> int:
        """Get **MPOS** (maximum position) [0‑4095]."""
        return self._read_12(self.REG_MPOS_MSB)

    def write_mpos(self, val: int):
        """Set **MPOS** – burn later for NVM."""
        self._write_12(self.REG_MPOS_MSB, val)
        
    def read_mang(self) -> int:
        """Get **MANG** (maximum position) [0‑4095]."""
        return self._read_12(self.REG_MANG_MSB)

    def write_mang(self, val: int):
        """Set **MANG** – burn later for NVM."""
        self._write_12(self.REG_MANG_MSB, val)        

    # ── public CONF API (getters / setters) ───────────────────────────────
    def get_power_mode(self) -> PowerMode:
        """Return current **PowerMode** enum."""
        return PowerMode(self._conf_bits(self._PM_MASK, 0))

    def set_power_mode(self, mode: PowerMode):
        """Set power mode.

        Args:
            mode (PowerMode): one of :pydata:`PowerMode.NOM` … `LPM3`
        """
        self._set_conf_bits(self._PM_MASK, 0, int(mode))

    def get_hysteresis(self) -> Hysteresis:
        """Return hysteresis setting (enum)."""
        return Hysteresis(self._conf_bits(self._HYST_MASK, 2))

    def set_hysteresis(self, hyst: Hysteresis):
        """Set output hysteresis: *OFF* or 1‑3 LSB."""
        self._set_conf_bits(self._HYST_MASK, 2, int(hyst))

    def get_output_stage(self) -> OutputStage:
        return OutputStage(self._conf_bits(self._OUTS_MASK, 4))

    def set_output_stage(self, stage: OutputStage):
        """Pick analog full‑swing, analog 10‑90 %, or PWM."""
        self._set_conf_bits(self._OUTS_MASK, 4, int(stage))

    def get_pwm_freq(self) -> PWMFreq:
        return PWMFreq(self._conf_bits(self._PWMF_MASK, 6))

    def set_pwm_freq(self, fq: PWMFreq):
        """Set PWM base frequency **115 → 920 Hz**."""
        self._set_conf_bits(self._PWMF_MASK, 6, int(fq))

    def get_slow_filter(self) -> SlowFilter:
        return SlowFilter(self._conf_bits(self._SF_MASK, 8))

    def set_slow_filter(self, filt: SlowFilter):
        """Pick CORDIC slow‑filter depth 16× … 2×."""
        self._set_conf_bits(self._SF_MASK, 8, int(filt))

    def get_fast_filter(self) -> FastFilterThreshold:
        return FastFilterThreshold(self._conf_bits(self._FTH_MASK, 10))

    def set_fast_filter(self, thresh: FastFilterThreshold):
        """Set fast‑filter threshold (see datasheet table)."""
        self._set_conf_bits(self._FTH_MASK, 10, int(thresh))

    def get_watchdog(self) -> bool:
        return bool(self._conf_bits(self._WD_MASK, 13))

    def set_watchdog(self, enable: bool):
        """Enable (True) or disable (False) the 1.6 s watchdog."""
        self._set_conf_bits(self._WD_MASK, 13, 1 if enable else 0)
        
    # ── Output registers (getters ) ──────────────────────────────
    def read_raw_anble(self) -> int:
        """Get **RAW_ANBLE** (maximum position) [0‑4095]."""
        return self._read_12(self.REG_RAW_ANGLE_MSB)
    
    def read_angle(self) -> int:
        """Get **ANGLE** (maximum position) [0‑4095]."""
        return self._read_12(self.REG_ANGLE_MSB)
    
    # ── Status registers (getters ) ───────────────────────────────
    def read_status(self) -> list:
        """Devuelve una lista con los códigos de estado activos."""
        status = self._read_byte(self.REG_STATUS)
        codes = []
        if status & self._MH:
            codes.append("MH")
        if status & self._ML:
            codes.append("ML")
        if status & self._MD:
            codes.append("MD")
        return codes
    
    def read_agc(self) -> int:
        """Read AGC (Automatic Gain Control) [0-255]."""
        return self._read_byte(self.REG_AGC)
    
    def read_magnitude(self) -> int:
        """Get **MAGNITUDE** (maximum position) [0‑4095]."""
        return self._read_12(self.REG_MAG_MSB)
    
    # ── Burn registers (setters ) ───────────────────────────────    
    def burn_setting(self):
        """Burns MANG and CONF to permanent memory (only if ZMCO==0)."""
        self._write_byte(self.REG_BURN, self._BURN_SETTING)
