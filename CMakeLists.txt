cmake_minimum_required(VERSION 3.16)
project(camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 修改点：OpenCV 4.4+ 中 SIFT 已移入主模块 features2d，不再需要 features2d
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui calib3d features2d)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Concurrent OpenGL SerialPort)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization features filters surface)

add_definitions(-DENABLE_PCL_VISUALIZATION)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")           
    add_compile_options(-Wno-deprecated-declarations -Wno-cast-function-type)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/ui)
include_directories(${CMAKE_SOURCE_DIR}/include/core)
include_directories(${CMAKE_SOURCE_DIR}/include/io)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

set(SOURCES
    src/main.cpp
    src/ui/mainwindow.cpp
    src/ui/calibration_io.cpp
    src/ui/mainwindow_tab1.cpp
    src/ui/mainwindow_tab2.cpp
    src/ui/mainwindow_tab3.cpp
    src/ui/mainwindow_tab4.cpp
    src/ui/mainwindow_tab5.cpp
    src/ui/mainwindow_roi.cpp
    src/ui/videowidget.cpp
    src/io/camerathread.cpp
    src/core/cameracalibration.cpp
    src/core/lasercalibration.cpp
    src/io/laserworker.cpp
    src/core/pointcloudbuilder.cpp
    src/core/pointcloudviewer.cpp
    src/io/serialportmanager.cpp
    src/core/pointcalibrator.cpp
    src/core/rotatingcalibrator.cpp
)

set(HEADERS
    include/ui/mainwindow.h
    include/ui/videowidget.h
    include/io/camerathread.h
    include/core/cameracalibration.h
    include/core/lasercalibration.h
    include/io/laserworker.h
    include/core/pointcloudbuilder.h
    include/core/pointcloudviewer.h
    include/io/serialportmanager.h
    include/core/pointcalibrator.h
    include/core/rotatingcalibrator.h
)

add_executable(camera ${SOURCES} ${HEADERS})

target_link_libraries(camera 
    Qt5::Core 
    Qt5::Gui
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::OpenGL
    Qt5::SerialPort
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

set_target_properties(camera PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
