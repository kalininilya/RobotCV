cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project( webcamtest-tracking )
find_package( OpenCV REQUIRED )

#QT5
# cmake example from http://qt-project.org/doc/qt-5.0/qtdoc/cmake-manual.html
# Find the QtWidgets library

set(CMAKE_AUTOMOC ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5SerialPort REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

source_group( Autogen FILES ${Robot_Autogen_SOURCE_LIST} )

add_executable( webcamtest-tracking ${Robot_Autogen_SOURCE_LIST}
    ProcessingThread.cpp  
    TimingsDebug.h     
    WebcamCapture.h
    ProcessingThread.h 
    webcamtest.cpp
    TSDataHandler.h
    DebugOutput.h   
    targetver.h           
    WebcamCapture.cpp
 )
target_link_libraries( webcamtest-tracking ${OpenCV_LIBS})
qt5_use_modules(webcamtest-tracking Core)
