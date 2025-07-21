cmake_minimum_required(VERSION 3.10)
project(autobot)
find_package(ament_cmake REQUIRED)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()