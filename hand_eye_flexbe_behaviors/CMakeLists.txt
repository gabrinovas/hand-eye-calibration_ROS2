cmake_minimum_required(VERSION 3.5)
project(hand_eye_flexbe_behaviors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Instalar manifiestos en múltiples ubicaciones
install(DIRECTORY
  manifest
  DESTINATION share/${PROJECT_NAME}/manifest
)

install(DIRECTORY
  manifest
  DESTINATION lib/${PROJECT_NAME}/${PROJECT_NAME}/manifest
)

install(DIRECTORY
  manifest
  DESTINATION lib/${PROJECT_NAME}/manifest
)

# Instalar el paquete Python
ament_python_install_package(${PROJECT_NAME})

ament_package()
