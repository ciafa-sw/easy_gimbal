from dataclasses import dataclass


class GimbalCommand:
    """Base class for typified commands"""


@dataclass
class ZoomSpeed(GimbalCommand):
    zoom_in: bool
    speed: int


@dataclass
class ZoomPos(GimbalCommand):
    pos: int


@dataclass
class SensorChange(GimbalCommand):
    sensor_id: int


@dataclass
class ResolutionChange(GimbalCommand):
    resolution: int


@dataclass
class PanTiltSpeed(GimbalCommand):
    pan_speed: int
    tilt_speed: int


@dataclass
class PanTiltPos(GimbalCommand):
    pan_pos: int
    tilt_pos: int


class GimbalResponse:
    """Base class for typified commands"""


@dataclass
class GimbalTelemetry(GimbalResponse):
    data: bytes


@dataclass
class GimbalCommandError(GimbalResponse):
    """generic gimbal command error"""

