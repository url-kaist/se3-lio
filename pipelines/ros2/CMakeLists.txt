cmake_minimum_required(VERSION 3.16...3.26)

unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
unset(CATKIN_SYMLINK_INSTALL)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

set(SE3_LIO_CPP_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../cpp/se3_lio/install)
set(SE3_LIO_CPP_DIR ${PROJECT_SOURCE_DIR}/../cpp/se3_lio)
if(EXISTS ${SE3_LIO_CPP_INSTALL_DIR})
    message(STATUS "Using installed SE3-LIO C++ core library from ${SE3_LIO_CPP_INSTALL_DIR}...")
    link_directories(${SE3_LIO_CPP_INSTALL_DIR}/lib/se3_lio)
    add_executable(lio_node src/lio_node.cpp)
    ament_target_dependencies(lio_node
        rclcpp
        nav_msgs
        sensor_msgs
        geometry_msgs
        pcl_conversions
        livox_ros_driver2
        tf2_ros
    )
    target_link_libraries(lio_node
        core synchronizer pipeline Sophus::Sophus
    )
    target_include_directories(lio_node
        PRIVATE
        ${SE3_LIO_CPP_INSTALL_DIR}/include/se3_lio
    )
    install(DIRECTORY
        launch
        config
        rviz
        DESTINATION share/${PROJECT_NAME}
    )

    install(TARGETS lio_node
        DESTINATION lib/
    )
elseif (EXISTS ${SE3_LIO_CPP_DIR})
    message(STATUS "Building SE3-LIO C++ core library from source...")
    add_subdirectory(${PROJECT_SOURCE_DIR}/../cpp/se3_lio ${CMAKE_CURRENT_BINARY_DIR}/se3_lio_cpp)
    include_directories(${PROJECT_SOURCE_DIR}/../cpp/se3_lio)

    add_executable(lio_node src/lio_node.cpp)

    target_link_libraries(lio_node
        se3_lio::synchronizer se3_lio::pipeline
    )
    ament_target_dependencies(lio_node
        rclcpp
        nav_msgs
        sensor_msgs
        geometry_msgs
        pcl_conversions
        livox_ros_driver2
        tf2_ros
    )
    target_include_directories(lio_node
        PRIVATE
        ${PCL_INCLUDE_DIRS}
    )

    install(DIRECTORY
        launch
        config
        rviz
        DESTINATION share/${PROJECT_NAME}
    )

    install(TARGETS lio_node
        DESTINATION lib/${PROJECT_NAME}
    )
else()
    message(FATAL_ERROR "SE3-LIO C++ core library not found. Please build the C++ core first.")
endif()

ament_package()

