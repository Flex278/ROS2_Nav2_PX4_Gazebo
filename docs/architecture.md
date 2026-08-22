# Архитектура цифрового двойника UAV

## Общая схема

```
[ Хост Linux Mint (Экран + GTX 1050) ]
       ▲                                 │
       │ (проброс GPU/графики)           │ (команды управления)
       │                                 ▼
┌────────────────── DOCKER КОНТЕЙНЕР ──────────────────┐
│                                                      │
│  1. Gazebo Harmonic — рендер мира и физика           │
│         │ (видеопоток, одометрия, физика)            │
│         ▼                                            │
│  2. PX4 SITL — полётный контроллер                   │
│         │ (uORB)                                     │
│         ▼                                            │
│  3. MicroXRCEAgent — мост uORB ↔ ROS2                │
│         │ (топики /fmu/in, /fmu/out, /camera, ...)   │
│         ▼                                            │
│  4. ПО-ноды (Python 3.12):                           │
│     - uav_offboard   (MPC/Безье → trajectory_setpoint)│
│     - uav_perception (YOLO / H-маркер → /detections) │
│     - uav_navigation (Nav2 → /goal_pose, /cmd_vel)   │
│     - uav_sensors    (ros_gz_bridge, depth→scan)     │
└──────────────────────────────────────────────────────┘
```

## Пакеты ROS2 и их связи

| Пакет | Вход (подписки) | Выход (публикации) |
|-------|-----------------|--------------------|
| `uav_sensors` | Gazebo (через ros_gz_bridge) | `/camera/image_raw`, `/camera/depth`, `/camera/points`, `/scan` |
| `uav_offboard` | `/fmu/out/vehicle_local_position`, `/fmu/out/vehicle_status` | `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` |
| `uav_perception` | `/camera/image_raw` | `/detections` (uav_msgs/TargetDetection) |
| `uav_navigation` | `/scan`, `/tf`, `/detections` | `/goal_pose`, `/cmd_vel` |

## Ключевые инженерные решения

1. **QoS для PX4.** Топики `px4_msgs` — `Best Effort` + `Durability: VOLATILE`.
   Несовпадение QoS — самая частая причина «глухого» дрона.
2. **venv для CV.** `opencv-python`/`ultralytics`/`torch` ставятся в отдельный
   `venv/`, чтобы не конфликтовать с системным `rclpy`.
3. **Рендер.** В контейнере нет NVIDIA GL — рендер через Mesa/llvmpipe на
   DISPLAY=:0 (без `__NV_PRIME_RENDER_OFFLOAD`). Тени отключены, физика настроена
   под GTX 1050.
4. **3D-навигация.** Nav2 плоский; высота удерживается через `OffboardControlMode`
   (position+z). VoxelLayer (облако точек) отключён — перегружает однопоточный
   executor; для 2D-навигации достаточно obstacle_layer (depth->scan).
5. **Воспроизводимость.** PX4 зафиксирован как git submodule на конкретном коммите.
