cmake_minimum_required(VERSION 3.10.0)

project(CamtoRobotArm VERSION 0.1.0 LANGUAGES C CXX)


set(CMAKE_CXX_STANDARD 20)

# 경로 설정
set(OpenCV_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/lib/opencv/include")
set(OpenCV_LIB_DIR "${CMAKE_SOURCE_DIR}/lib/opencv/bin")
set(DETECTOR_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include/Detectors")
set(CALIB_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include/Calib")
set(Include_DIR "${CMAKE_SOURCE_DIR}/include")
set(EIGEN_DIR "${CMAKE_SOURCE_DIR}/lib/Eigen/Eigen")

# include 경로 등록
include_directories(
    ${OpenCV_INCLUDE_DIR}
    ${Include_DIR}
    ${DETECTOR_INCLUDE_DIR}
    ${CALIB_INCLUDE_DIR}
    ${EIGEN_DIR}
    )

# 라이브러리 경로 등록
link_directories(${OpenCV_LIB_DIR} ${YAML_CPP_INCLUDE_DIR})

# 실행 파일 생성
add_executable(CamtoRobotArm
    main.cpp
    src/Logger.cpp
    src/Detectors/ArUcoMakerDetector.cpp
    src/Detectors/QRcodeDetector.cpp
    src/Calib/Calibrator.cpp
    src/Robot/RobotArm.cpp
    src/Robot/openRobotArm.cpp
    src/yamlReader.cpp
)

# 필요한 OpenCV 모듈 수동 연결 (필요한 .a/.so 파일 이름만 작성)
target_link_libraries(CamtoRobotArm
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_imgcodecs
    opencv_videoio
    opencv_calib3d
    opencv_objdetect
    opencv_aruco
    yaml-cpp
)