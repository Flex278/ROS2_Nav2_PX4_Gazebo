# Карта топиков ROS2

> Канал PX4 ↔ ROS2 работает через MicroXRCE-DDS (**без MAVROS**).
> Все топики `px4_msgs` используют QoS **Best Effort + VOLATILE**.

## ROS2 ↔ PX4

| Топик | Тип (`px4_msgs`) | Направление |
|-------|------------------|-------------|
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | PX4 → ROS2 |
| `/fmu/out/vehicle_odometry` | `VehicleOdometry` | PX4 → ROS2 |
| `/fmu/out/vehicle_status` | `VehicleStatus` | PX4 → ROS2 (arming/mode) |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | PX4 → ROS2 |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | ROS2 → PX4 |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | ROS2 → PX4 |
| `/fmu/in/vehicle_command` | `VehicleCommand` | ROS2 → PX4 (arm/disarm/mode) |

## Сенсоры (Gazebo → ROS2 через `ros_gz_bridge`)

| Топик | Тип | Назначение |
|-------|-----|------------|
| `/camera/image_raw` | `sensor_msgs/Image` | RGB для YOLO / H-маркера |
| `/camera/depth` | `sensor_msgs/Image` | глубина для VIO / OctoMap |
| `/camera/points` | `sensor_msgs/PointCloud2` | облако точек |
| `/scan` | `sensor_msgs/LaserScan` | выход `depth_to_scan.py` → Nav2 |

## Автономность

| Топик | Тип | Назначение |
|-------|-----|------------|
| `/detections` | `uav_msgs/TargetDetection` | результат CV (bbox + класс + confidence) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | цель для Nav2 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → offboard (пересчёт в setpoint) |
| `/tf`, `/tf_static` | TF2 | калибровка камеры, base_link → map |
