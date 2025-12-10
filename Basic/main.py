import sys
import time
import threading
from pymavlink import mavutil
from drone_monitor import DroneState, monitor_loop
from mission_control import clear_mission, upload_mission, MissionItem
from flight_control import arm, takeoff, land, set_mode_guided, set_mode_auto

def connect(connection_string: str = "tcp:127.0.0.1:14550") -> mavutil.mavlink_connection:
    master = mavutil.mavlink_connection(connection_string)
    result = master.wait_heartbeat(timeout=1)
    if result is None:
        return None
    print(f"Подключено к системе {master.target_system}, компонент {master.target_component}")
    return master

def wait_for_coordinates(state: DroneState, timeout: float = 15.0) -> bool:
    print("Ожидаем текущие координаты дрона...")
    end_time = time.time() + timeout
    while time.time() < end_time:
        if state.last_update > 0 and not (state.lat_deg == 0.0 and state.lon_deg == 0.0):
            print(f"Координаты получены: lat={state.lat_deg:.7f}, lon={state.lon_deg:.7f}, alt_rel={state.alt_rel_m:.1f} м")
            return True
        time.sleep(1.0)
    print("Не удалось получить координаты.")
    return False

def create_single_waypoint_mission_from_state(state: DroneState, target_alt_m: float) -> list[MissionItem]:
    lat_int = int(state.lat_deg * 1e7)
    lon_int = int(state.lon_deg * 1e7)
    wp = MissionItem(
        seq=0,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=1,
        autocontinue=1,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        x=lat_int,
        y=lon_int,
        z=target_alt_m,
    )
    return [wp]

def wait_for_altitude(state: DroneState, target_alt_m: float, tolerance_m: float = 1.0, timeout: float = 30.0) -> bool:
    print(f"Ждём набора высоты ~{target_alt_m} м...")
    end_time = time.time() + timeout
    while time.time() < end_time:
        alt = state.alt_rel_m
        print(f"  текущая высота: {alt:.1f} м")
        if alt >= target_alt_m - tolerance_m:
            print("Целевая высота достигнута.")
            return True
        time.sleep(1.0)
    print("Высота не достигнута.")
    return False

if __name__ == "__main__":
    master = connect()
    if master is None:
        print("Проверьте подключение.")
        sys.exit(1)
    state = DroneState()
    stop_flag = {"stop": False}
    monitor_thread = threading.Thread(
        target=monitor_loop,
        args=(master, state, lambda: stop_flag["stop"]),
        daemon=False,
    )
    monitor_thread.start()
    print("Поток мониторинга запущен.")
    try:
        if not wait_for_coordinates(state):
            sys.exit(1)
        TARGET_ALT_M = 20.0
        print("Очищаем старую миссию и загружаем новую из одной точки...")
        clear_mission(master)
        mission_items = create_single_waypoint_mission_from_state(state, TARGET_ALT_M)
        print("Останавливаем мониторинг на время загрузки миссии...")
        stop_flag["stop"] = True
        monitor_thread.join(timeout=2.0)
        print("Мониторинг остановлен, загружаем миссию.")
        upload_mission(master, mission_items)
        print("Миссия загружена.")
        stop_flag = {"stop": False}
        monitor_thread = threading.Thread(
            target=monitor_loop,
            args=(master, state, lambda: stop_flag["stop"]),
            daemon=False,
        )
        monitor_thread.start()
        print("Мониторинг снова запущен.")
        print("Переводим дрон в GUIDED, ARM, взлёт и затем AUTO.")
        set_mode_guided(master)
        print("Пытаемся сделать ARM двигателей...")
        arm(master)
        for _ in range(10):
            if state.armed:
                print("Дрон ARM")
                break
            time.sleep(1.0)
        else:
            print("Не удалось включить ARM.")
            sys.exit(1)
        print(f"Команда TAKEOFF до {TARGET_ALT_M} м...")
        takeoff(master, TARGET_ALT_M)
        if not wait_for_altitude(state, TARGET_ALT_M, timeout=40.0):
            sys.exit(1)
        set_mode_auto(master)
        print("Висим в точке 10 секунд...")
        for i in range(10):
            print(f"t={i:02d}s  alt={state.alt_rel_m:.1f} м  armed={state.armed}  mode='{state.mode}'")
            time.sleep(1.0)
        print("Команда LAND...")
        land(master)
    finally:
        print("Останавливаем поток мониторинга...")
        stop_flag["stop"] = True
        monitor_thread.join(timeout=2.0)
        print("Программа завершается.")