cmake_minimum_required(VERSION 3.5)
project(bcr_bot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY launch meshes models urdf worlds rviz
    DESTINATION share/${PROJECT_NAME}/
)

install( DIRECTORY config/
         DESTINATION share/${PROJECT_NAME}/config)
         
         
install(PROGRAMS scripts/remapper.py
        scripts/goal_publisher.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
