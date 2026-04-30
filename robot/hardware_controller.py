"""
Hardware controller for RC car via Arduino over serial.

Converts normalized PID outputs (-1 to 1) to PWM values and sends over serial.
"""

from __future__ import annotations

import logging
import serial
import threading
import time
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class SteeringConfig:
    """Configuration for steering servo PWM."""
    center: int = 1500  # Center/neutral position in µs
    range_us: int = 500  # Range from center in µs (left: center - range, right: center + range)
    
    def get_pwm(self, normalized_value: float) -> int:
        """Convert normalized value (-1 to 1) to PWM microseconds.
        
        -1.0 = full left (center - range)
        0.0 = center
        1.0 = full right (center + range)
        """
        pwm = self.center + int(normalized_value * self.range_us)
        # Clamp to safe range
        return max(1000, min(2000, pwm))


@dataclass
class ThrottleConfig:
    """Configuration for throttle/ESC PWM."""
    idle: int = 1500  # Idle/neutral position in µs
    max_throttle: int = 2000  # Maximum throttle in µs
    
    def get_pwm(self, normalized_value: float) -> int:
        """Convert normalized value (0 to 1) to PWM microseconds.
        
        0.0 = idle
        1.0 = max throttle
        
        Note: Negative values are clamped to idle (no reverse).
        """
        # Clamp normalized value to 0-1 range
        clamped = max(0.0, min(1.0, normalized_value))
        pwm = self.idle + int(clamped * (self.max_throttle - self.idle))
        return max(1000, min(2000, pwm))


class ArduinoController:
    """Serial interface to Arduino for RC car control."""
    
    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 9600, timeout: float = 1.0):
        """Initialize Arduino controller.
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0' on Linux, 'COM3' on Windows)
            baud: Baud rate (default 9600)
            timeout: Serial timeout in seconds
        """
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self._lock = threading.Lock()
        self._last_steer_pwm: Optional[int] = None
        self._last_throttle_pwm: Optional[int] = None
        
        self.steering_config = SteeringConfig()
        self.throttle_config = ThrottleConfig()
    
    def connect(self) -> bool:
        """Connect to Arduino over serial.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
            )
            time.sleep(2)  # Give Arduino time to reset and initialize
            self.connected = True
            logger.info(f"Connected to Arduino on {self.port} at {self.baud} baud")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> None:
        """Disconnect from Arduino."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.connected = False
            logger.info("Disconnected from Arduino")
    
    def _send_command(self, command: str) -> bool:
        """Send a command to the Arduino.
        
        Args:
            command: Command string (without newline)
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.connected or not self.serial or not self.serial.is_open:
            logger.warning("Not connected to Arduino")
            return False
        
        try:
            with self._lock:
                self.serial.write((command + "\n").encode())
            return True
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            self.connected = False
            return False
    
    def set_steering(self, normalized_value: float) -> bool:
        """Set steering servo position.
        
        Args:
            normalized_value: -1.0 (full left) to 1.0 (full right)
            
        Returns:
            True if sent successfully
        """
        pwm = self.steering_config.get_pwm(normalized_value)
        if self._last_steer_pwm == pwm:
            return True
        command = f"STEER:{pwm}"
        sent = self._send_command(command)
        if sent:
            self._last_steer_pwm = pwm
        return sent
    
    def set_throttle(self, normalized_value: float) -> bool:
        """Set throttle/ESC power.
        
        Args:
            normalized_value: 0.0 (idle) to 1.0 (max throttle)
            Negative values are clamped to idle.
            
        Returns:
            True if sent successfully
        """
        pwm = self.throttle_config.get_pwm(normalized_value)
        if self._last_throttle_pwm == pwm:
            return True
        command = f"THROTTLE:{pwm}"
        sent = self._send_command(command)
        if sent:
            self._last_throttle_pwm = pwm
        return sent
    
    def stop(self) -> bool:
        """Stop the car (set steering and throttle to idle).
        
        Returns:
            True if sent successfully
        """
        sent = self._send_command("STOP")
        if sent:
            self._last_steer_pwm = self.steering_config.center
            self._last_throttle_pwm = self.throttle_config.idle
        return sent
    
    def update_steering_config(self, center: int, range_us: int) -> None:
        """Update steering configuration.
        
        Args:
            center: Center/neutral position in µs
            range_us: Range from center in µs
        """
        self.steering_config.center = center
        self.steering_config.range_us = range_us
        logger.info(f"Updated steering config: center={center}, range={range_us}")
    
    def update_throttle_config(self, idle: int, max_throttle: int) -> None:
        """Update throttle configuration.
        
        Args:
            idle: Idle position in µs
            max_throttle: Maximum throttle position in µs
        """
        self.throttle_config.idle = idle
        self.throttle_config.max_throttle = max_throttle
        logger.info(f"Updated throttle config: idle={idle}, max={max_throttle}")
