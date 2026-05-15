cmake_minimum_required(VERSION 3.5)
project(franka_impedance_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/cartesian_impedance_controller.cpp)
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
        ${franka_semantic_components_INCLUDE_DIRS}
)
target_link_libraries(
        ${PROJECT_NAME}
        PUBLIC
        Franka::Franka
        ${franka_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${rcl_interfaces_TARGETS}
        ${std_msgs_TARGETS}
        ${franka_semantic_components_LIBRARIES}
        controller_interface::controller_interface
        hardware_interface::hardware_interface
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
)

if(${hardware_interface_VERSION} VERSION_GREATER_EQUAL "4.20")
    target_compile_definitions(${PROJECT_NAME} PRIVATE HW_HAS_SET_NODISCARD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wno-maybe-uninitialized)
endif()

pluginlib_export_plugin_description_file(
        controller_interface franka_impedance_controller.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        franka_msgs
        franka_semantic_components
        geometry_msgs
        hardware_interface
        pluginlib
        rcl_interfaces
        rclcpp
        rclcpp_lifecycle
        std_msgs
)
ament_package()
