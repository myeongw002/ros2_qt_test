cmake_minimum_required(VERSION 3.8)
project(qt_gui_test)

# 기본 컴파일러 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# Qt 자동화 설정 (UI, MOC, RCC 처리)
set(CMAKE_AUTOUIC ON)  # .ui 파일 자동 변환
set(CMAKE_AUTOMOC ON)  # Q_OBJECT 매크로 자동 처리
set(CMAKE_AUTORCC ON)  # Qt 리소스 파일 자동 처리
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Qt UI 파일이 있는 경로 추가
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# ROS2 및 Qt5 라이브러리 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)




# 실행 파일 소스 목록
add_executable(${PROJECT_NAME} 
    src/main.cpp
    src/mainwindow.cpp
    ui/mainwindow.ui
)

# MOC 파일 자동 생성
set(MOC_HEADERS include/qt_gui_test/mainwindow.h)
qt5_wrap_cpp(MOC_SRCS ${MOC_HEADERS})

# include/ 디렉토리 추가 (헤더 파일을 찾기 위해 필요)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
)

# Qt 및 ROS2 라이브러리 링크
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    Qt5Core
    Qt5Widgets
)

# MOC 파일을 빌드 대상에 추가
target_sources(${PROJECT_NAME} PRIVATE ${MOC_SRCS})

# 실행 파일을 ROS2 패키지로 설치
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

