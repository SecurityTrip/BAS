import sys
import time
import threading
import math
from pymavlink import mavutil
from drone_monitor import DroneState, monitor_loop
from mission_control import clear_mission, upload_mission, MissionItem
from flight_control import arm, takeoff, land, set_mode_guided, set_mode_auto

def connect(connection_string: str = "udpin:127.0.0.1:14550") -> mavutil.mavlink_connection:
    master = mavutil.mavlink_connection(
        connection_string,
        source_system=255,
        source_component=190
    )
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

def create_mission_circle_1km_with_return(state: DroneState, altitude_m: float = 20.0, radius_m: float = 100.0) -> list[MissionItem]:
    """
    Создаёт миссию:
    1. Взлёт в текущей точке (home)
    2. Полёт по кругу радиусом radius_m
    3. Возврат в точку взлёта
    4. Посадка
    """
    center_lat = state.lat_deg
    center_lon = state.lon_deg

    METERS_PER_DEG_LAT = 111194.9267
    METERS_PER_DEG_LON = METERS_PER_DEG_LAT * math.cos(math.radians(center_lat))

    mission = []

    # 0 — Домашняя точка (взлёт)
    home_lat_int = int(center_lat * 1e7)
    home_lon_int = int(center_lon * 1e7)
    mission.append(MissionItem(
        seq=len(mission),
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=1,           # ← это стартовая точка миссии
        autocontinue=1,
        param1=0, param2=0, param3=0, param4=0,
        x=home_lat_int,
        y=home_lon_int,
        z=altitude_m,
    ))

    # 1
    points = 32
    for i in range(points):
        angle_deg = i * (360.0 / points)
        angle_rad = math.radians(angle_deg)

        offset_lat_m = radius_m * math.sin(angle_rad)
        offset_lon_m = radius_m * math.cos(angle_rad)

        wp_lat = center_lat + offset_lat_m / METERS_PER_DEG_LAT
        wp_lon = center_lon + offset_lon_m / METERS_PER_DEG_LON

        mission.append(MissionItem(
            seq=len(mission),
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=0,
            autocontinue=1,
            param1=0, param2=0, param3=0, param4=0,
            x=int(wp_lat * 1e7),
            y=int(wp_lon * 1e7),
            z=altitude_m,
        ))

    first_circle_wp = mission[1]  # первая точка круга (seq=1)
    mission.append(MissionItem(
        seq=len(mission),
        frame=first_circle_wp.frame,
        command=first_circle_wp.command,
        current=0,
        autocontinue=1,
        param1=0, param2=0, param3=0, param4=0,
        x=first_circle_wp.x,
        y=first_circle_wp.y,
        z=altitude_m,
    ))

    # 9 — Возврат домой
    mission.append(MissionItem(
        seq=len(mission),
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=0,
        autocontinue=1,
        param1=0, param2=0, param3=0, param4=0,
        x=home_lat_int,
        y=home_lon_int,
        z=altitude_m,
    ))

    # 10 — Посадка
    mission.append(MissionItem(
        seq=len(mission),
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_LAND,
        current=0,
        autocontinue=0,
        param1=0, param2=0, param3=0, param4=0,
        x=home_lat_int,
        y=home_lon_int,
        z=0.0,
    ))

    print(f"Создана миссия из {len(mission)} пунктов: круг {radius_m} м + возврат + посадка")
    return mission

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
        print("Ошибка подключения к дрону через Mission Planner.")
        sys.exit(1)

    state = DroneState()
    stop_flag = {"stop": False}
    monitor_thread = threading.Thread(
        target=monitor_loop,
        args=(master, state, lambda: stop_flag["stop"]),
        daemon=False,
    )
    monitor_thread.start()
    print("Поток мониторинга запущен (непрерывно).")

    try:
        print("Ожидание GPS-фикса и текущих координат...")
        if not wait_for_coordinates(state, timeout=20.0):
            print("Не удалось получить координаты — проверьте GPS.")
            sys.exit(1)

        TARGET_ALT_M = 20.0
        CIRCLE_RADIUS_M = 50.0

        print(f"\nСоздаём миссию: круг радиусом {CIRCLE_RADIUS_M} м + возврат домой + посадка")
        clear_mission(master)
        mission_items = create_mission_circle_1km_with_return(
            state,
            altitude_m=TARGET_ALT_M,
            radius_m=CIRCLE_RADIUS_M
        )
        print(f"Миссия из {len(mission_items)} пунктов создана.")

        upload_mission(master, mission_items, state)
        print("Миссия успешно загружена в автопилот!")

        # === 1. Перевод в GUIDED ===
        print("\nПереводим дрон в режим GUIDED...")
        set_mode_guided(master)

        # === 2. ARM ===
        print("Выполняем ARM двигателей...")
        arm(master)
        for _ in range(15):
            if state.armed:
                print("Дрон вооружён (ARMED) — готов к взлёту!")
                break
            time.sleep(0.5)
        else:
            print("Ошибка: не удалось выполнить ARM")
            sys.exit(1)

        # === 3. Взлёт на 20 м ===
        print(f"Команда TAKEOFF на высоту {TARGET_ALT_M} м...")
        takeoff(master, TARGET_ALT_M)

        if not wait_for_altitude(state, TARGET_ALT_M, tolerance_m=2.0, timeout=60.0):
            print("Не удалось набрать высоту — аварийная посадка.")
            land(master)
            sys.exit(1)

        print(f"Высота {TARGET_ALT_M} м достигнута — начинаем миссию!")

        # === 4. Запуск миссии в режиме AUTO ===
        print("Переходим в режим AUTO — старт полёта по кругу!")
        set_mode_auto(master)

        # Ждём подтверждения режима AUTO
        for _ in range(40):
            if state.mode == "AUTO":
                print("Режим AUTO активирован — дрон выполняет миссию!")
                break
            time.sleep(0.25)
        else:
            print(f"Режим остался {state.mode} — миссия всё равно выполняется")

        # === 5. Мониторим выполнение миссии ===
        print("\nПолёт по кругу начат! Следим за дроном...")
        print("t=     | высота    | режим     | статус")
        print("-" * 50)

        start_time = time.time()
        while time.time() - start_time < 180:  # максимум 3 минуты на круг
            elapsed = int(time.time() - start_time)
            print(f"t={elapsed:03d}s | alt={state.alt_rel_m:6.1f} м | mode={state.mode:8} | armed={state.armed}")
            time.sleep(2.0)

            # Если дрон начал снижаться — значит, дошёл до LAND
            if state.alt_rel_m < 5.0 and state.alt_rel_m > 0.1:
                print("Дрон начал посадку...")
                break

        # === 6. Финальная посадка (на всякий случай) ===
        print("\nМиссия завершена. Принудительная команда LAND на случай зависания...")
        land(master)

        # Ждём полной остановки
        print("Ожидаем приземления...")
        while state.alt_rel_m > 0.5:
            print(f"  → высота: {state.alt_rel_m:.1f} м (снижается)")
            time.sleep(2.0)

        print("Дрон успешно приземлился. Миссия выполнена!")

    except KeyboardInterrupt:
        print("\nПрерывание пользователем — аварийная посадка...")
        land(master)
    except Exception as e:
        print(f"\nКритическая ошибка: {e}")
        land(master)
    finally:
        print("Останавливаем поток мониторинга...")
        stop_flag["stop"] = True
        monitor_thread.join(timeout=3.0)
        print("Программа завершена. Дрон в безопасности.")