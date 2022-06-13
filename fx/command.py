from abc import ABC, abstractmethod
import math
import struct
from enum import Enum

from pymavlink.fx.mav import MavSerializer

from pymavlink import mavutil


class AircraftModeEnum(Enum):
    MANUAL = 0
    CIRCLE = 1
    STABILIZE = 2
    TRAINING = 3
    ACRO = 4
    FBWA = 5
    FBWB = 6
    CRUISE = 7
    AUTOTUNE = 8
    AUTO = 10
    RTL = 11
    LOITER = 12
    TAKEOFF = 13
    AVOID_ADSB = 14
    GUIDED = 15
    INITIALISING = 16
    QSTABILIZE = 17
    QHOVER = 18
    QLOITER = 19
    QLAND = 20
    QRTL = 21
    QAUTOTUNE = 22
    QACRO = 23
    THERMAL = 24


class Command(ABC):
    def __init__(self, mav_serializer: MavSerializer = None):
        if mav_serializer is None:
            mav_serializer = MavSerializer()
        self.ms = mav_serializer

    @abstractmethod
    def serialize_payload(self, *args, **kwargs) -> bytes:
        pass


class Context:
    def __init__(
        self, command: Command, use_fx_format: bool = False, delimiter: bytes = None
    ) -> None:
        self._command = command
        self._use_fx_format = use_fx_format
        self._delimiter = delimiter

    @property
    def delimiter(self) -> bytes:
        return self._delimiter

    @delimiter.setter
    def delimiter(self, delimiter: bytes) -> None:
        self._delimiter = delimiter

    @property
    def use_fx_format(self) -> bool:
        return self._use_fx_format

    @use_fx_format.setter
    def use_fx_format(self, use_fx_format: bool) -> None:
        self._use_fx_format = use_fx_format

    @property
    def command(self) -> Command:
        return self._command

    @command.setter
    def command(self, command: Command) -> None:
        self._command = command

    def get_payload(self, *args, **kwargs):
        payload = self._command.serialize_payload(*args, **kwargs)
        if not self._use_fx_format:
            return payload
        if self._delimiter:
            payload += self._delimiter
        return struct.pack(
            ">HH{}s".format(len(payload)),
            len(bytes([self._command.ms.target_system]) + payload),
            self._command.ms.target_system,
            payload,
        )


class HeartbeatCommand(Command):
    def serialize_payload(self, *args, **kwargs) -> bytes:
        payload = self.ms.mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )
        return payload.pack(self.ms.mav)


class ArmCommand(Command):
    def serialize_payload(self, *args, **kwargs) -> bytes:
        if "arm" not in kwargs:
            raise TypeError("Missing arm value")
        payload = self.ms.mav.command_long_encode(
            self.ms.target_system,
            self.ms.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
            0,  # confirmation
            int(kwargs["arm"]),  # param 1
            0,  # param 2
            0,  # param 3
            0,  # param 4
            0,  # param 5
            0,  # param 6
            0,  # param 7
        )
        return payload.pack(self.ms.mav)


class ChangeAltitudeCommand(Command):
    def serialize_payload(self, *args, **kwargs) -> bytes:
        if "alt" not in kwargs:
            raise TypeError("Missing alt value")
        payload = self.ms.mav.mission_item_encode(
            self.ms.target_system,
            self.ms.target_component,
            0,  # sequence
            0,  # frame MAV_FRAME_GLOBAL
            16,  # cmd MAV_CMD_NAV_WAYPOINT
            3,  # current. Contrary to the official documentation, current IS NOT a boolean (╯°□°)╯︵ ┻━┻. A value of 3 means change of altitude only
            0,  # autocontinue
            0,  # param 1
            0,  # param 2
            0,  # param 3
            0,  # param 4
            0,  # lat
            0,  # lon
            kwargs["alt"],  # alt
        )
        return payload.pack(self.ms.mav)


class ChangeHeadingCommand(Command):
    def serialize_payload(self, *args, **kwargs) -> bytes:
        if not all(
            [True if x in kwargs else False for x in ["lon", "lat", "heading", "alt"]]
        ):
            raise TypeError("Missing lon, lat, heading or alt value")
        Nm_2_km = 1.852
        rad_2_deg = 180 / math.pi
        distance = 150 * Nm_2_km
        lon_new, lat_new = self.calculate_destination(
            kwargs["lon"], kwargs["lat"], kwargs["heading"], distance
        )
        payload = self.ms.mav.mission_item_encode(
            self.ms.target_system,
            self.ms.target_component,
            0,  # sequence
            0,  # frame MAV_FRAME_GLOBAL
            16,  # cmd MAV_CMD_NAV_WAYPOINT
            2,  # current. NOT A BOOLEAN (╯°□°)╯︵ ┻━┻. 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
            1,  # autocontinue
            0,  # param 1
            0,  # param 2
            0,  # param 3
            0,  # param 4
            lat_new * rad_2_deg,  # lat
            lon_new * rad_2_deg,  # lon
            kwargs["alt"],  # alt
        )
        return payload.pack(self.ms.mav)

    @staticmethod
    def calculate_destination(lon_start, lat_start, heading, d):
        R = 6378.1  # Radius of the Earth (km)

        lat_end = lat_start - d / R * math.cos(heading)
        lon_end = lon_start + d / R * math.sin(heading)

        return lon_end, lat_end


class SetWaypointCommand(Command):
    def serialize_payload(self, *args, **kwargs) -> bytes:
        if "waypoint_id" not in kwargs:
            raise TypeError("Missing waypoint_id value")
        payload = self.ms.mav.mission_set_current_encode(
            self.ms.target_system, self.ms.target_component, int(kwargs["waypoint_id"])
        )
        return payload.pack(self.ms.mav)


class SetModeCommand(Command):
    def serialize_payload(self, *args, **kwargs) -> bytes:
        if "mode" not in kwargs:
            raise TypeError("Missing mode value")
        payload = self.ms.mav.set_mode_encode(
            self.ms.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            int(kwargs["mode"]),
        )
        return payload.pack(self.ms.mav)


class GetAllParamsCommand(Command):
    def serialize_payload(self, *args, **kwargs) -> bytes:
        payload = self.ms.mav.param_request_list_encode(
            self.ms.target_system, self.ms.target_component
        )
        return payload.pack(self.ms.mav)


class SetParamCommand(Command):
    def serialize_payload(self, *args, **kwargs) -> bytes:
        if not all([True if x in kwargs else False for x in ["name", "value", "type"]]):
            raise TypeError("Missing name, value, type values")
        payload = self.ms.mav.param_set_encode(
            self.ms.target_system,
            self.ms.target_component,
            kwargs["name"].encode("utf-8"),
            kwargs["value"],
            kwargs["type"]
            if isinstance(kwargs["type"], int)
            else getattr(mavutil.mavlink, kwargs["type"]),
        )
        return payload.pack(self.ms.mav)


class GetParamCommand(Command):
    def serialize_payload(self, *args, **kwargs) -> bytes:
        if "name" not in kwargs:
            raise TypeError("Missing name value")
        payload = self.ms.mav.param_request_read_encode(
            self.ms.target_system,
            self.ms.target_component,
            kwargs["name"].encode("utf-8"),
            -1,
        )
        return payload.pack(self.ms.mav)


class PreflightCalibrationCommand(Command):
    def serialize_payload(self, *args, **kwargs) -> bytes:
        payload = self.ms.mav.command_long_encode(
            self.ms.target_system,
            self.ms.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1
            0,  # param 2
            1,  # param 3
            0,  # param 4
            0,  # param 5
            0,  # param 6
            0,  # param 7
        )
        return payload.pack(self.ms.mav)
