from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Protocol

from field_coordinate.config import FieldConfig


class ReachabilityLed(Protocol):
    def set_reachable(self, reachable: bool) -> None:
        ...


class TargetReporter(Protocol):
    def report(self, *, lat_deg: float, lon_deg: float) -> None:
        ...


@dataclass
class NullLed:
    def set_reachable(self, reachable: bool) -> None:
        return None


@dataclass
class GpioLed:
    pin: int
    gpio: object

    def __post_init__(self) -> None:
        self.gpio.setmode(self.gpio.BCM)
        self.gpio.setwarnings(False)
        self.gpio.setup(self.pin, self.gpio.OUT)

    def set_reachable(self, reachable: bool) -> None:
        value = self.gpio.HIGH if reachable else self.gpio.LOW
        self.gpio.output(self.pin, value)


@dataclass
class NullTargetReporter:
    def report(self, *, lat_deg: float, lon_deg: float) -> None:
        return None


@dataclass
class I2CTargetReporter:
    bus: object
    gpio: object | None = None
    write_address: int = 80
    register: int = 0
    gpio_pin: int = 3

    def report(self, *, lat_deg: float, lon_deg: float) -> None:
        payload = struct.pack("<ff", float(lat_deg), float(lon_deg))
        self.bus.write_i2c_block_data(self.write_address, self.register, list(payload))
        if self.gpio is not None:
            self.gpio.setmode(self.gpio.BCM)
            self.gpio.setup(self.gpio_pin, self.gpio.OUT)
            self.gpio.output(self.gpio_pin, self.gpio.HIGH)


def create_led_from_config(config: FieldConfig) -> ReachabilityLed:
    if not config.gpio_enabled:
        return NullLed()
    try:
        import RPi.GPIO as GPIO
    except ImportError as exc:
        raise RuntimeError("GPIO output requested but RPi.GPIO is not installed") from exc
    return GpioLed(pin=config.reachable_led_pin, gpio=GPIO)


def create_target_reporter_from_config(config: FieldConfig) -> TargetReporter:
    if not config.i2c_enabled:
        return NullTargetReporter()
    try:
        from smbus2 import SMBus
    except ImportError as exc:
        raise RuntimeError("I2C output requested but smbus2 is not installed") from exc
    bus = SMBus(config.i2c_bus)
    gpio = None
    if config.gpio_enabled:
        try:
            import RPi.GPIO as GPIO
        except ImportError as exc:
            raise RuntimeError("GPIO output requested but RPi.GPIO is not installed") from exc
        gpio = GPIO
    return I2CTargetReporter(
        bus=bus,
        gpio=gpio,
        write_address=config.i2c_write_address,
        register=config.i2c_register,
        gpio_pin=config.found_gpio_pin,
    )
