cmake_minimum_required(VERSION 3.16...3.26)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    pcl_conversions
    pcl_ros
    std_msgs
    sensor_msgs
    message_generation
    livox_ros_driver2
)

find_package(PCL REQUIRED)
catkin_package()

set(SE3_LIO_CPP_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../cpp/se3_lio/install)
set(SE3_LIO_CPP_DIR ${PROJECT_SOURCE_DIR}/../cpp/se3_lio)
if(EXISTS ${SE3_LIO_CPP_INSTALL_DIR})
    message(STATUS "Using installed SE3-LIO C++ core library from ${SE3_LIO_CPP_INSTALL_DIR}...")
    link_directories(${SE3_LIO_CPP_INSTALL_DIR}/lib/se3_lio)

    add_executable(lio_node src/lio_node.cpp)
    target_link_libraries(lio_node ${catkin_LIBRARIES} core synchronizer pipeline Sophus::Sophus)
    target_include_directories(lio_node
        PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${SE3_LIO_CPP_INSTALL_DIR}/include/se3_lio
    )
elseif (EXISTS ${SE3_LIO_CPP_DIR})
    message(STATUS "Building SE3-LIO C++ core library from source...")
    add_subdirectory(
        ${PROJECT_SOURCE_DIR}/../cpp/se3_lio
        ${CMAKE_CURRENT_BINARY_DIR}/se3_lio
    )
    include_directories(${PROJECT_SOURCE_DIR}/../cpp)

    add_executable(lio_node src/lio_node.cpp)
    target_link_libraries(lio_node ${catkin_LIBRARIES} se3_lio::synchronizer se3_lio::pipeline)
    target_include_directories(lio_node
        PRIVATE
        ${catkin_INCLUDE_DIRS}
    )

else()
    message(FATAL_ERROR "SE3-LIO C++ core library not found. Please build the C++ core first.")
endif()