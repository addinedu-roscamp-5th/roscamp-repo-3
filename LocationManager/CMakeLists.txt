
cmake_minimum_required(VERSION 3.10)
project(my_project)


# OpenCV 경로 직접 지정
# set(OpenCV_DIR "/home/addinedu/for_opencv/build")

# 패키지 찾기
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

#실행 파일 생성
add_executable(GwanjeCam main.cpp src/GwanjeCam.cpp) 



# 링크할 라이브러리
target_link_libraries(GwanjeCam
    rclcpp::rclcpp
   ${std_msgs_TARGETS}
    ${OpenCV_LIBS}
)



