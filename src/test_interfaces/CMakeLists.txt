# CMake 3.8 이상 필요
cmake_minimum_required(VERSION 3.8) 

# 프로젝트 이름
project(test_interfaces)

# 컴파일 옵션(경고 설정)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ROS2의 빌드 시스템(ament_cmake) 찾기
find_package(ament_cmake REQUIRED)

# msg / srv/ action을 코드로 바꿔줌 
find_package(rosidl_default_generators REQUIRED)

# action 메시지 의존성
find_package(action_msgs REQUIRED)

# 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Position.msg"
    "srv/CalculateDistance.srv"
    "action/MoveTo.action"
    DEPENDENCIES action_msgs
)

# 이 프로젝트를 ROS2 패키지로 등록함. ( 이 코드가 없으면 패키지가 아님. )
ament_package()

