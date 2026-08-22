#!/usr/bin/env bash
# ============================================================
# setup_all.sh — полная автономная подготовка окружения UAV Digital Twin.
# Идемпотентный и продолжаемый: выполненные шаги помечаются маркером в
# .setup_state/, при повторном запуске сделанные шаги пропускаются.
#
#   Запуск:  nohup bash scripts/setup_all.sh > setup_all.log 2>&1 &
#   Лог:     setup_all.log       Состояние: .setup_state/
# ============================================================
set -u
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"
STATE_DIR="$PROJECT_ROOT/.setup_state"; mkdir -p "$STATE_DIR"
LOG="$PROJECT_ROOT/setup_all.log"
PX4_TAG="v1.15.4"; PX4_BRANCH="release/1.15"
PX4_URL="https://github.com/PX4/PX4-Autopilot.git"
PX4_DIR="$PROJECT_ROOT/PX4-Autopilot"
IMAGE="uav-dt:jazzy"; CONTAINER="uav_dt_jazzy"
WS="/workspaces/ROS2_Nav2_PX4_Gazebo"

log()  { echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
is_done() { [ -f "$STATE_DIR/$1.done" ]; }
mark() { touch "$STATE_DIR/$1.done"; log "OK: шаг '$1' завершён"; }
err()  { log "ОШИБКА: $*"; }

log "=== setup_all.sh СТАРТ ==="

# ---------- 01: клонирование PX4-Autopilot ----------
if is_done 01_px4_clone; then
  log "пропуск 01_px4_clone (уже сделано)"
else
  log ">>> 01: клонирование PX4-Autopilot (тег $PX4_TAG, рекурсивно)"
  if [ ! -d "$PX4_DIR/.git" ]; then
    git clone --depth 1 -b "$PX4_TAG" "$PX4_URL" "$PX4_DIR" >>"$LOG" 2>&1
    if [ $? -ne 0 ]; then
      err "clone $PX4_TAG не удался — пробую ветку $PX4_BRANCH"
      rm -rf "$PX4_DIR"
      git clone --depth 1 -b "$PX4_BRANCH" "$PX4_URL" "$PX4_DIR" >>"$LOG" 2>&1
    fi
  fi
  if [ -d "$PX4_DIR/.git" ]; then
    ( cd "$PX4_DIR" && git submodule update --init --recursive --depth 1 >>"$LOG" 2>&1 ) \
      && mark 01_px4_clone \
      || err "submodules не доинициализированы (повторный запуск продолжит)"
    # Мелкий (--depth 1) клон не содержит тегов, а сборка PX4 генерирует
    # версию из git-тегов (в т.ч. тегов nuttx-* в сабмодуле NuttX) и падает
    # без них с IndexError в px_update_git_header.py.
    ( cd "$PX4_DIR" \
        && git fetch --tags --depth 1 origin >>"$LOG" 2>&1 \
        && git submodule foreach --recursive 'git fetch --tags --depth 1 origin 2>/dev/null || true' >>"$LOG" 2>&1 ) \
      || err "теги git не докачаны (сборка версии может упасть)"
  else
    err "PX4-Autopilot отсутствует"
  fi
fi

# ---------- 02: сборка Docker-образа ----------
if is_done 02_image; then
  log "пропуск 02_image (уже сделано)"
elif [ -f "$PROJECT_ROOT/.devcontainer/Dockerfile" ]; then
  log ">>> 02: docker build $IMAGE (базовый образ + Nav2 + ros_gz, ДОЛГО)"
  docker build -t "$IMAGE" -f "$PROJECT_ROOT/.devcontainer/Dockerfile" "$PROJECT_ROOT" >>"$LOG" 2>&1 \
    && mark 02_image || err "docker build не удался"
else
  err "нет .devcontainer/Dockerfile"
fi
# ---------- 03: запуск контейнера ----------
if is_done 03_container; then
  log "пропуск 03_container (уже сделано)"
elif docker image inspect "$IMAGE" >/dev/null 2>&1; then
  log ">>> 03: запуск контейнера $CONTAINER"
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  # --gpus all + DISPLAY + X11 — чтобы работал GUI Gazebo (как в devcontainer.json).
  docker run -d --name "$CONTAINER" --network host --privileged --gpus all \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PROJECT_ROOT:$WS" -w "$WS" "$IMAGE" sleep infinity >>"$LOG" 2>&1 \
    && mark 03_container || err "docker run не удался"
else
  err "образа $IMAGE нет — пропускаю шаг 03"
fi

# ---------- 04: сборка рабочего пространства ----------
if is_done 04_ws_build; then
  log "пропуск 04_ws_build (уже сделано)"
elif docker inspect "$CONTAINER" >/dev/null 2>&1; then
  log ">>> 04: colcon build --symlink-install"
  docker exec "$CONTAINER" bash -lc \
    "source /opt/ros/jazzy/setup.bash && cd $WS && colcon build --symlink-install" >>"$LOG" 2>&1 \
    && mark 04_ws_build || err "colcon build не удался"
else
  err "контейнера $CONTAINER нет — пропускаю шаг 04"
fi
# ---------- 05: venv для CV ----------
if is_done 05_venv; then
  log "пропуск 05_venv (уже сделано)"
elif docker inspect "$CONTAINER" >/dev/null 2>&1; then
  log ">>> 05: venv CV (opencv + ultralytics + torch CUDA, ДОЛГО)"
  docker exec "$CONTAINER" bash -c "cd $WS && bash scripts/setup_venv.sh" >>"$LOG" 2>&1 \
    && mark 05_venv || err "setup_venv не удался (CV; не блокирует остальное)"
else
  err "контейнера $CONTAINER нет — пропускаю шаг 05"
fi

# ---------- 06: тесты + валидация launch ----------
if is_done 06_tests; then
  log "пропуск 06_tests (уже сделано)"
else
  log ">>> 06: локальные тесты + валидация launch-файла"
  ( cd "$PROJECT_ROOT/tests" && for t in test_*.py; do
      python3 "$t" >/dev/null 2>&1 && log "PASS $t" || log "FAIL $t"
    done )
  if docker inspect "$CONTAINER" >/dev/null 2>&1; then
    docker exec "$CONTAINER" bash -lc \
      "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && ros2 launch uav_bringup sim_full.launch.py --show-args" >>"$LOG" 2>&1 \
      && log "show-args OK" || err "show-args FAIL"
  fi
  mark 06_tests
fi

# ---------- 07: предсборка PX4 SITL ----------
if is_done 07_px4_build; then
  log "пропуск 07_px4_build (уже сделано)"
elif docker inspect "$CONTAINER" >/dev/null 2>&1 && [ -d "$PX4_DIR/.git" ]; then
  log ">>> 07: предсборка PX4 SITL (make px4_sitl_default, ДОЛГО)"
  docker exec "$CONTAINER" bash "$WS/scripts/build_px4.sh" >>"$LOG" 2>&1 \
    && mark 07_px4_build || err "PX4 build не удался"
else
  err "нет контейнера или PX4 — пропускаю шаг 07"
fi

# ---------- 08: Micro XRCE DDS Agent (мост uORB <-> ROS2, из исходников) ----------
if is_done 08_agent; then
  log "пропуск 08_agent (уже сделано)"
elif docker inspect "$CONTAINER" >/dev/null 2>&1; then
  log ">>> 08: сборка Micro XRCE DDS Agent (из исходников)"
  docker exec "$CONTAINER" bash "$WS/scripts/build_agent.sh" >>"$LOG" 2>&1 \
    && mark 08_agent || err "агент Micro XRCE DDS не собрался (мост uORB<->ROS2 будет недоступен)"
else
  err "контейнера $CONTAINER нет — пропускаю шаг 08"
fi


# ---------- ИТОГ ----------
log "=== ИТОГ ==="
ALL_OK=1
for s in 01_px4_clone 02_image 03_container 04_ws_build 05_venv 06_tests 07_px4_build 08_agent; do
  if is_done "$s"; then log "  [done] $s"; else log "  [MISS] $s"; ALL_OK=0; fi
done
if [ "$ALL_OK" = 1 ]; then
  touch "$STATE_DIR/ALL_DONE"
  log "ВСЁ ГОТОВО. Утром: bash scripts/run_sim.sh  (GUI)  или  ros2 launch uav_bringup sim_full.launch.py headless:=true"
else
  log "Не все шаги выполнены — перезапустите: bash scripts/setup_all.sh"
fi
log "=== setup_all.sh КОНЕЦ ==="


