# ✅ Успешный запуск — референс (21.08.2026)

**Состояние:** полный стек (PX4 + MicroXRCEAgent + Gazebo headless + GUI + Nav2 + миссия) работает стабильно. Дрон взлетает, обходит препятствия, выполняет миссию «Патруль улицы» (28 целей) и возвращается.

> **Важно:** этот файл фиксирует *рабочее* состояние перед экспериментами с высотой и коллизиями. Если что-то сломается — откатывайся к этим параметрам.

---

## 1. Запуск (порядок)

```bash
# ─── Терминал 1: headless-сервер + PX4 + Nav2 ───
docker exec -it uav_mission bash

source /opt/ros/jazzy/setup.bash
source /workspaces/ROS2_Nav2_PX4_Gazebo/install/setup.bash

# Скрипт запускает: ros2 launch uav_bringup sim_full.launch.py
#   world:=street.world mission_mode:=nav2 takeoff_height:=3.0 headless:=true
bash scripts/launch_sim_headless.sh street.world

# ─── Терминал 2: GUI окно (на хосте :0) ───
docker exec uav_mission bash scripts/launch_gz_gui_x0.sh

# ─── Терминал 3: миссия «Патруль улицы» ───
docker exec uav_mission bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /workspaces/ROS2_Nav2_PX4_Gazebo/install/setup.bash
  cd /workspaces/ROS2_Nav2_PX4_Gazebo
  python3 scripts/nav2_mission_street.py --speed 1.0
'
```

---

## 2. Ключевые параметры (рабочие)

### 2.1. Высота взлёта

| Файл | Значение |
|---|---|
| `scripts/launch_sim_headless.sh` (стр. 35) | `takeoff_height:=3.0` |
| `src/uav_bringup/launch/sim_full.launch.py` (стр. 108) | `takeoff_height = context.launch_configurations.get('takeoff_height', '3.0')` |
| `src/uav_bringup/launch/sim_full.launch.py` (стр. 355) | `default_value='3.0'` |
| `src/uav_offboard/config/nav2_commander.yaml` (стр. 4) | `takeoff_height: 3.0` |

> **Фактически:** дрон летит на **3.0 м**.

### 2.2. Модели машин (коллизия)

| Модель | Файл | Высота коллизии |
|---|---|---|
| `car_blue` | `src/uav_gazebo/models/car_blue/model.sdf:63-64` | `pose z=0.5`, `box 4.5×1.8×1.0` → верх коллизии **1.0 м** |
| `car_green` | `src/uav_gazebo/models/car_green/model.sdf:63-64` | то же |
| `car_red` | `src/uav_gazebo/models/car_red/model.sdf:63-64` | то же |

> **Зазор:** дрон 3.0 м − верх машины 1.0 м = **2.0 м** — безопасно.

### 2.3. Мир

| Параметр | Значение |
|---|---|
| Файл | `src/uav_gazebo/worlds/street.world` |
| Копируется в | `PX4-Autopilot/Tools/simulation/gz/worlds/street.sdf` (автоматически при launch) |
| Небо | `<scene><background>0.53 0.81 0.92</background></scene>` — **голубое**, `<sky>` удалён |
| Физика | ODE, `max_step_size=0.004`, `RTF=1.0`, `update_rate=250` |

### 2.4. Timesync (рабочий, но нестабильный)

- `ENABLE_LOCKSTEP_SCHEDULER=ON` (PX4 `sitl.cmake:14`)
- `GZBridge` дёргает `px4_clock_settime(CLOCK_MONOTONIC)` на каждом такте 250 Гц
- Иногда timesync сбрасывается («time jump detected» → `reset_filter()`)
- **НО:** стартовый цикл `while (_synchronize_timestamps)` успевает пройти → PX4 загружается
- Workaround если зациклится: `docker exec uav_mission bash -lc 'ros2 param set /uxrce_dds_client UXRCE_DDS_SYNCT 0'`

---

## 3. Проверка состояния (снапшот)

```bash
# Процессы (должны быть без дубликатов и defunct)
docker exec uav_mission ps aux | grep -E 'gz sim|parameter_bridge|bin/px4|MicroXRCEAgent|nav2_commander|controller_server|planner_server|bt_navigator' | grep -v grep

# PX4 статус (armed=2, nav_state=14 OFFBOARD)
docker exec uav_mission bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic echo /fmu/out/vehicle_status --qos-reliability best_effort --once 2>&1 | grep -E "arming_state|nav_state|failsafe|pre_flight"'

# Позиция (xy_valid=true, z_valid=true, z≈-3.0)
docker exec uav_mission bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>&1 | grep -E "x:|y:|z:|xy_valid|z_valid"'
```

---

## 4. Nav2

| Компонент | Состояние |
|---|---|
| `controller_server` | active |
| `planner_server` | active |
| `behavior_server` | active |
| `bt_navigator` | active |
| `waypoint_follower` | active |
| `lifecycle_manager_navigation` | «Managed nodes are active» |

---

## 5. Убийство процессов

```bash
# Только через контейнер! pkill с хоста не имеет прав на root-процессы контейнера.
docker exec uav_mission bash scripts/_kill_sim_tmp.sh
```

---

## 6. Файлы, которые НЕ трогать без необходимости

| Файл | Причина |
|---|---|
| `PX4-Autopilot/Tools/simulation/gz/worlds/street.sdf` | Генерируется автоматически при launch из `src/uav_gazebo/worlds/street.world` |
| `PX4-Autopilot/build/` | Сборка PX4, пересобирается только при изменении исходников PX4 |
| `install/` | `--symlink-install` → ссылается на `src/`, пересобирается через `colcon build` |

---

## 7. История правок (что уже сделано)

1. ✅ Удалён `<sky><time>10.0</time></sky>` из `street.world` и `5pillars.world` — небо голубое
2. ✅ Идентифицирована root-cause timesync-зацикливания (lockstep + CLOCK_MONOTONIC)
3. ✅ Найден workaround: `UXRCE_DDS_SYNCT=0`
4. ✅ Запуск работает: PX4 → MicroXRCEAgent → Gazebo → Nav2 → миссия
5. ✅ Высота взлёта поднята до 3.0 м, коллизии машин уменьшены до 1.0 м — зазор 2.0 м