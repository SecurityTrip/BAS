from dataclasses import dataclass
from typing import List
from pymavlink import mavutil
import time

@dataclass
class MissionItem:
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float

def clear_mission(master: mavutil.mavlink_connection) -> None:
    master.mav.mission_clear_all_send(
        master.target_system,
        master.target_component
    )

def upload_mission(master: mavutil.mavlink_connection, items: List[MissionItem], state) -> None:  # Добавлен state
    count = len(items)
    if count == 0:
        return
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        count
    )
    sent = 0
    last_seq = -1
    last_ack = -1
    timeout = time.time() + 30  # Общий таймаут
    while sent < count and time.time() < timeout:
        # Ожидаем изменения в DroneState вместо recv_match
        if state.last_mission_request_seq != last_seq:
            seq = state.last_mission_request_seq
            last_seq = seq
            if seq < 0 or seq >= count:
                continue
            item = items[seq]
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                item.seq,
                item.frame,
                item.command,
                item.current,
                item.autocontinue,
                item.param1,
                item.param2,
                item.param3,
                item.param4,
                item.x,
                item.y,
                item.z,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            sent += 1
        if state.last_mission_ack_type != last_ack:
            last_ack = state.last_mission_ack_type
            if last_ack == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("MISSION_ACK: миссия принята.")
                break
            else:
                print(f"MISSION_ACK: ошибка {last_ack}.")
                raise ValueError(f"Ошибка загрузки миссии: {last_ack}")
        time.sleep(0.1)  # Небольшая пауза для проверки состояния
    if sent < count:
        raise TimeoutError("Таймаут загрузки миссии.")

def download_mission(master: mavutil.mavlink_connection) -> List[MissionItem]:
    # Без изменений, так как не требует рефакторинга для непрерывности
    master.mav.mission_request_list_send(
        master.target_system,
        master.target_component
    )
    msg = master.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=5)
    if msg is None:
        return []
    count = msg.count
    items: List[MissionItem] = []
    for seq in range(count):
        master.mav.mission_request_int_send(
            master.target_system,
            master.target_component,
            seq,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        item_msg = master.recv_match(type=['MISSION_ITEM_INT'], blocking=True, timeout=5)
        if item_msg is None:
            continue
        items.append(
            MissionItem(
                seq=item_msg.seq,
                frame=item_msg.frame,
                command=item_msg.command,
                current=item_msg.current,
                autocontinue=item_msg.autocontinue,
                param1=item_msg.param1,
                param2=item_msg.param2,
                param3=item_msg.param3,
                param4=item_msg.param4,
                x=item_msg.x,
                y=item_msg.y,
                z=item_msg.z,
            )
        )
    return items