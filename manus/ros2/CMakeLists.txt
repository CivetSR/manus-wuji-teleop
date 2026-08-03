cmake_minimum_required(VERSION 3.11)
project(manus_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED ENV{MANUS_SDK})
  set(MANUS_SDK "$ENV{HOME}/ManusSDK")
else()
  set(MANUS_SDK "$ENV{MANUS_SDK}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manus_ros2_msgs REQUIRED)

add_executable(manus_data_publisher
  src/manus_data_publisher.cpp
  src/ClientPlatformSpecific.cpp
  src/ManusDataPublisher.cpp)
target_include_directories(manus_data_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MANUS_SDK}/include)
target_link_directories(manus_data_publisher PRIVATE ${MANUS_SDK}/lib)

target_link_libraries(manus_data_publisher ManusSDK_Integrated ncurses usb-1.0 udev z pthread)
target_compile_features(manus_data_publisher PUBLIC c_std_99 cxx_std_17)

install(TARGETS manus_data_publisher DESTINATION lib/${PROJECT_NAME})

set(LIBRARY_FILE ${MANUS_SDK}/lib/libManusSDK_Integrated.so)
install(FILES ${LIBRARY_FILE} DESTINATION lib/${PROJECT_NAME})
set_target_properties(manus_data_publisher PROPERTIES INSTALL_RPATH "$ORIGIN")

ament_target_dependencies(manus_data_publisher rclcpp std_msgs geometry_msgs manus_ros2_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
