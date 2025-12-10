import time
from pymavlink import mavutil

def set_mode(master: mavutil.mavlink_connection, mode_name: str, timeout: float = 5.0) -> None:
    mode_mapping = master.mode_mapping()
    if mode_mapping is None or mode_name not in mode_mapping:
        raise ValueError(f"Режим {mode_name} недоступен")
    mode_id = mode_mapping[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    end_time = time.time() + timeout
    while time.time() < end_time:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if msg:
            break

def set_mode_guided(master: mavutil.mavlink_connection) -> None:
    set_mode(master, "GUIDED")

def set_mode_auto(master: mavutil.mavlink_connection) -> None:
    set_mode(master, "AUTO")

def arm(master: mavutil.mavlink_connection, force: bool = False) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0 if not force else 21196,
        0, 0, 0, 0, 0
    )

def disarm(master: mavutil.mavlink_connection, force: bool = False) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0 if not force else 21196,
        0, 0, 0, 0, 0
    )

def takeoff(master: mavutil.mavlink_connection, alt_m: float) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0,
        0,
        alt_m
    )

def land(master: mavutil.mavlink_connection) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0,
        0, 0, 0
    )