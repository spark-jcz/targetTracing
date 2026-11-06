cmake_minimum_required(VERSION 3.5)
project(video_subscriber_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(videoSubscriber src/videoSubscriber.cpp)
ament_target_dependencies(videoSubscriber rclcpp sensor_msgs std_msgs)
add_executable(videoSubscriberH264 src/videoSubscriberH264.cpp)
ament_target_dependencies(videoSubscriberH264 rclcpp sensor_msgs std_msgs)
target_include_directories(videoSubscriber PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(videoSubscriber ${OpenCV_LIBS})

install(TARGETS videoSubscriber videoSubscriberH264
  DESTINATION lib/${PROJECT_NAME})

ament_package()
