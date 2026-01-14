# CMake 최소 요구 버전 설정
cmake_minimum_required(VERSION 3.8)

# 프로젝트(패키지) 이름 설정
project(gps)

# ROS2 빌드 시스템(ament)과 메시지 생성기(rosidl)를 찾음
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# msg 폴더에 있는 .msg 파일을 기반으로 인터페이스(파이썬 코드 등)를 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleState.msg"
)

# ament_python 패키지 설치 규칙
find_package(ament_python REQUIRED)
ament_python_install_package()

# ROS2 패키지 설정 마무리
ament_package()
