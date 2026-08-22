# Цель миссии для Nav2 / offboard-ноды.

std_msgs/Header header          # Временная метка и frame_id

string mission_type             # "takeoff" | "waypoint" | "land" | "track_target"

geometry_msgs/PoseStamped target_pose   # Целевая поза (для waypoint / land)

float32 yaw                     # Курс [rad]
float32 height                  # Заданная высота полёта [м]
