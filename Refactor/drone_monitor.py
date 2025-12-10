from dataclasses import dataclass
import time
from pymavlink import mavutil

@dataclass
class DroneState:
    last_update: float = 0.0
    mode: str = ""
    armed: bool = False
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_rel_m: float = 0.0
    battery_voltage_v: float = 0.0
    battery_remaining_pct: float = 0.0
    last_mission_request_seq: int = -1  # Новый: Последний запрошенный seq миссии
    last_mission_ack_type: int = -1     # Новый: Тип последнего ACK (0=ACCEPTED, etc.)

def _handle_heartbeat(master, msg, state: DroneState) -> None:
    state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    mode_mapping = master.mode_mapping()
    if mode_mapping:
        inv_mode_mapping = {v: k for k, v in mode_mapping.items()}
        mode_id = msg.custom_mode
        state.mode = inv_mode_mapping.get(mode_id, f"UNKNOWN({mode_id})")

def _handle_global_position_int(msg, state: DroneState) -> None:
    state.lat_deg = msg.lat / 1e7
    state.lon_deg = msg.lon / 1e7
    state.alt_rel_m = msg.relative_alt / 1000.0

def _handle_sys_status(msg, state: DroneState) -> None:
    if msg.voltage_battery > 0:
        state.battery_voltage_v = msg.voltage_battery / 1000.0
    state.battery_remaining_pct = float(msg.battery_remaining)

def _handle_mission_request_int(msg, state: DroneState) -> None:
    state.last_mission_request_seq = msg.seq

def _handle_mission_ack(msg, state: DroneState) -> None:
    state.last_mission_ack_type = msg.type

def monitor_loop(master: mavutil.mavlink_connection, state: DroneState, stop_flag_getter=lambda: False) -> None:
    while not stop_flag_getter():
        msg = master.recv_match(blocking=True, timeout=1)
        now = time.time()
        if msg is None:
            continue
        msg_type = msg.get_type()
        if msg_type == 'HEARTBEAT':
            _handle_heartbeat(master, msg, state)
        elif msg_type == 'GLOBAL_POSITION_INT':
            _handle_global_position_int(msg, state)
        elif msg_type == 'SYS_STATUS':
            _handle_sys_status(msg, state)
        elif msg_type == 'MISSION_REQUEST_INT':
            _handle_mission_request_int(msg, state)
        elif msg_type == 'MISSION_ACK':
            _handle_mission_ack(msg, state)
        state.last_update = now