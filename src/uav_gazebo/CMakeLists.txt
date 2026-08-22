cmake_minimum_required(VERSION 3.8)
project(uav_gazebo)

find_package(ament_cmake REQUIRED)

# Установка миров и моделей Gazebo.
# Для использования добавьте в окружение:
#   export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:<install>/uav_gazebo/share/uav_gazebo/models
install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
