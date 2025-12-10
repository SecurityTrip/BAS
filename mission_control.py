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

def upload_mission(master: mavutil.mavlink_connection, items: List[MissionItem], state) -> None:
    count = len(items)
    if count == 0:
        return

    state.last_mission_request_seq = -1
    state.last_mission_ack_type = -1

    print(f"Отправляю MISSION_COUNT = {count}")
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        count,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )

    sent = 0
    timeout = time.time() + 30

    while time.time() < timeout:
        time.sleep(0.02)

        if (state.last_mission_request_seq != -1 and
                count > state.last_mission_request_seq >= sent):

            seq = state.last_mission_request_seq
            item = items[seq]
            print(f"→ Отправляю waypoint {seq}")

            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                seq,
                item.frame,
                item.command,
                item.current,
                item.autocontinue,
                item.param1, item.param2, item.param3, item.param4,
                item.x, item.y, item.z,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            sent = seq + 1

        if state.last_mission_ack_type != -1 and sent >= count:
            if state.last_mission_ack_type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("MISSION_ACK: миссия принята.")
                return
            else:
                raise RuntimeError(f"Ошибка миссии: {state.last_mission_ack_type}")

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