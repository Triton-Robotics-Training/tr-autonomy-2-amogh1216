cmake_minimum_required(VERSION 3.5)
project(your_sol) 

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(robo_spin_sol src/robo_spin_sol.cpp src/robo_spin_sol.h)
ament_target_dependencies(robo_spin_sol rclcpp std_msgs sensor_msgs image_transport cv_bridge)

install(TARGETS
        robo_spin_sol
        DESTINATION lib/${PROJECT_NAME})


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_package()
