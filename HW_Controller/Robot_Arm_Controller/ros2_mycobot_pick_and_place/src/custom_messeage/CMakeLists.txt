cmake_minimum_required(VERSION 3.8)
project(custom_messeage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === 필요한 패키지 찾기 ===
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# === 서비스 정의 ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RobotArmRequest.srv"
)

# === 테스트 설정 (기존 유지) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === 런타임 export
ament_export_dependencies(rosidl_default_runtime)

# === 패키지 설정 마무리
ament_package()
