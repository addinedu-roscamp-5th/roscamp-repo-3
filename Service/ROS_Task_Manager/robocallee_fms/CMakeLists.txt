cmake_minimum_required(VERSION 3.10.0)
project(robocallee_fms VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include 디렉토리 설정
set(ADAPTERS_INCLUDE_DIR    "${CMAKE_SOURCE_DIR}/Adapters/include")
set(CORE_INCLUDE_DIR        "${CMAKE_SOURCE_DIR}/Core/include")
set(LIB_INCLUDE_DIR         "${CMAKE_SOURCE_DIR}/lib")
set(REQUEST_INCLUDE_DIR     "${CMAKE_SOURCE_DIR}/Manager/include")
set(INTERFACE_INCLUDE_DIR   "${CMAKE_SOURCE_DIR}/Interface/include")
set(TASK_INCLUDE_DIR        "${CMAKE_SOURCE_DIR}/Task/include")
set(TRAFFIC_INCLUDE_DIR     "${CMAKE_SOURCE_DIR}/Traffic/include")
set(LOGGER_INCLUDE_DIR      "${CMAKE_SOURCE_DIR}/../../Utile/Logger/include")
set(UTILE_INCLUDE_DIR       "${CMAKE_SOURCE_DIR}/../../Utile/include")
set(COMMON_INCLUDE_DIR      "${CMAKE_SOURCE_DIR}/../../Utile/Common/include")
set(YAML_INCLUDE_DIR        "${CMAKE_SOURCE_DIR}/../../Lib/YAML/include")
set(YAML_LIB_DIR            "${CMAKE_SOURCE_DIR}/../../Lib/YAML/bin")
set(OPENCV_INCLUDE_DIR      "${CMAKE_SOURCE_DIR}/../../Lib/opencv_r/include")
set(OPENCV_LIB_DIR          "${CMAKE_SOURCE_DIR}/../../Lib/opencv_r/bin")

include_directories(
  ${ADAPTERS_INCLUDE_DIR}
  ${CORE_INCLUDE_DIR}
  ${LIB_INCLUDE_DIR}
  ${REQUEST_INCLUDE_DIR}
  ${INTERFACE_INCLUDE_DIR}
  ${TASK_INCLUDE_DIR}
  ${TRAFFIC_INCLUDE_DIR}
  ${UTILE_INCLUDE_DIR}
  ${LOGGER_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
  ${YAML_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIR}
	)

  # ROS2 & 메시지 의존성
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)         
  find_package(tf2 REQUIRED)                   
  find_package(tf2_ros REQUIRED)               
  find_package(tf2_geometry_msgs REQUIRED)     
  find_package(rosidl_default_generators REQUIRED)
  find_package(nav_msgs REQUIRED)


# 서비스 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ShoeRequest.srv"
  "srv/DoneMsg.srv"
  "srv/RobotArmRequest.srv"
  "srv/CustomerRequest.srv"
  "srv/EmployeeRequest.srv"
  "msg/ArucoPose.msg"
  "msg/ArucoPoseArray.msg"
  DEPENDENCIES std_msgs
)


add_executable(robocallee_fms_node
    main.cpp
    ../../Utile/Logger/src/Logger.cpp
    Core/src/Core.cpp
    Task/src/Dispatcher.cpp
    Task/src/Executor.cpp
    Interface/src/RosInterface.cpp
    Adapters/src/AmrAdapter.cpp
    Adapters/src/RobotArmAdapter.cpp
    Manager/src/RequestManager.cpp
    Manager/src/StorageManager.cpp
    Traffic/src/OccupancyGrid.cpp
    Traffic/src/TrafficPlanner.cpp
)



# 자동 생성된 메시지 헤더 포함
target_include_directories(robocallee_fms_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
  )

#실행 파일(robocallee_fms_node)를 사용하는 ROS2 패키지들을 link 해줌
ament_target_dependencies(robocallee_fms_node
  rclcpp  
  std_msgs
  geometry_msgs            # ← 추가
  tf2                      # ← 추가
  tf2_ros                  # ← 추가 
  tf2_geometry_msgs        # ← 추가
  nav_msgs
)

# 링크 라이브러리
target_link_libraries(robocallee_fms_node
  ${OPENCV_LIB_DIR}/libopencv_world.so    
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${cpp_typesupport_target}
  ${YAML_LIB_DIR}/libyaml-cpp.so
)

#런타임시 동적라이브러리 경로 찾기
set_target_properties(robocallee_fms_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${OPENCV_LIB_DIR};${YAML_LIB_DIR};${CMAKE_INSTALL_PREFIX}/lib;${CMAKE_INSTALL_PREFIX}/lib/robocallee_fms"
)

install(TARGETS
  robocallee_fms_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()