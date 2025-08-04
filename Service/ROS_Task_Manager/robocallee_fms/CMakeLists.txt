cmake_minimum_required(VERSION 3.10.0)

project(robocallee_fms VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)

#GCC or Clang
#-Wall: 기본 경고 전부 표시
#-Wextra: 추가 경고
#-Wpedantic: 표준에 어긋나는 코드도 경고

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 경로 설정
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

find_package(ament_cmake REQUIRED)                  # ament 빌드시스템 사용
find_package(rclcpp REQUIRED)                       # ROS2 C++ 클라이언트 라이브러리
find_package(std_msgs REQUIRED)                     # 표준 메시지 타입들 (e.g. std_msgs::String)
find_package(rosidl_default_generators REQUIRED)    # 서비스 메시지 생성에 사용

# include 경로 등록
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

  # 서비스 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/ShoeRequest.srv"
    "srv/DoneMsg.srv"
    "srv/RobotArmRequest.srv"

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
    Traffic/src/OccupancyGrid.cpp
)

target_include_directories(robocallee_fms_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

#실행 파일(robocallee_fms_node)를 사용하는 ROS2 패키지들을 link 해줌
ament_target_dependencies(robocallee_fms_node
    rclcpp  
)

#필요한 OpenCV 모듈 수동 연결 (필요한 .a/.so 파일 이름만 작성)
target_link_libraries(robocallee_fms_node
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${YAML_LIB_DIR}/libyaml-cpp.so
    ${OPENCV_LIB_DIR}/libopencv_world.so
)

#런타임시 동적라이브러리 경로 찾기
set_target_properties(robocallee_fms_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH ${OPENCV_LIB_DIR}
  INSTALL_RPATH ${YAML_LIB_DIR}
)

install(TARGETS
  robocallee_fms_node
  DESTINATION lib/${PROJECT_NAME})


#메시지 타입이(자동 생성된 헤더)을 이 실행 파일(robocallee_fms_node)이 사용할 수 있도록 연결
# rosidl_target_interfaces(robocallee_fms_node
  # ${PROJECT_NAME} "rosidl_typesupport_cpp")


rosidl_get_typesupport_target(cpp_typesupport_target 
                             "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")





ament_package()