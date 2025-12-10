from dataclasses import dataclass
from typing import List
from pymavlink import mavutil

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

def upload_mission(master: mavutil.mavlink_connection, items: List[MissionItem]) -> None:
    count = len(items)
    if count == 0:
        return
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        count
    )
    sent = 0
    while sent < count:
        msg = master.recv_match(
            type=['MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True,
            timeout=5
        )
        if msg is None:
            continue
        if msg.get_type() == 'MISSION_REQUEST_INT':
            seq = msg.seq
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
        elif msg.get_type() == 'MISSION_ACK':
            print("MISSION_ACK получен, загрузка миссии завершена.")
            break

def download_mission(master: mavutil.mavlink_connection) -> List[MissionItem]:
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