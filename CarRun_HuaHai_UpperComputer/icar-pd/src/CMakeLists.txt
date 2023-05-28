cmake_minimum_required(VERSION 3.4...3.18)
project(patrol_car)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -mcpu=native -flto -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(INCLUDE_PATH "/usr/local/include")
set(LIB_PATH "/usr/local/lib")
#---------------------------------------------------------------------
#       [ Include、Lib  : Define ] ==> []
#---------------------------------------------------------------------

set(COMMON_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/")

set(COMMON_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include"
                        "${PROJECT_SOURCE_DIR}/config"
                        "${PROJECT_SOURCE_DIR}/demo" 
                        "${PROJECT_SOURCE_DIR}/driver" 
                        "${PROJECT_SOURCE_DIR}/lib" 
                        "${PROJECT_SOURCE_DIR}/src"
                        "${PROJECT_SOURCE_DIR}/tool"          )


link_directories(${COMMON_LIB_DIR})
include_directories( ${COMMON_INCLUDE_DIR} )
#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#---------------------------------------------------------------------
find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )    
include_directories( ${OpenCV_INCLUDE_DIRS} )

pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

# find ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

# find onnx
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})
#---------------------------------------------------------------------
#               [ bin ] ==> [ tool ]
#---------------------------------------------------------------------
# Image2video
# set(IMAGE2VIDEO_PROJECT_NAME "image2video")
# set(IMAGE2VIDEO_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/image2video.cpp)
# add_executable(${IMAGE2VIDEO_PROJECT_NAME} ${IMAGE2VIDEO_PROJECT_SOURCES})
# target_link_libraries(${IMAGE2VIDEO_PROJECT_NAME}  pthread)
# target_link_libraries(${IMAGE2VIDEO_PROJECT_NAME}  ${OpenCV_LIBS})

# CameraDisplay
set(CAMERA_DISPLAY_PROJECT_NAME "camera_display")
set(CAMERA_DISPLAY_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/camera_display.cpp)
add_executable(${CAMERA_DISPLAY_PROJECT_NAME} ${CAMERA_DISPLAY_PROJECT_SOURCES})
target_link_libraries(${CAMERA_DISPLAY_PROJECT_NAME}  pthread)
target_link_libraries(${CAMERA_DISPLAY_PROJECT_NAME}  ${OpenCV_LIBS})

# ImageCollection
set(COLLECTION_PROJECT_NAME "image_collection")
set(COLLECTION_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/image_collection.cpp)
add_executable(${COLLECTION_PROJECT_NAME} ${COLLECTION_PROJECT_SOURCES})
target_link_libraries(${COLLECTION_PROJECT_NAME}  pthread)
target_link_libraries(${COLLECTION_PROJECT_NAME}  ${OpenCV_LIBS})
target_link_libraries(${COLLECTION_PROJECT_NAME}  ${SERIAL_LIBRARIES})

# ImageCalibration（相机标定）
set(CALIBRATION_PROJECT_NAME "camera_calibration")
set(CALIBRATION_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/camera_calibration.cpp)
add_executable(${CALIBRATION_PROJECT_NAME} ${CALIBRATION_PROJECT_SOURCES})
target_link_libraries(${CALIBRATION_PROJECT_NAME}  pthread)
target_link_libraries(${CALIBRATION_PROJECT_NAME}  ${OpenCV_LIBS})
target_link_libraries(${CALIBRATION_PROJECT_NAME}  ${SERIAL_LIBRARIES})

# Ai测试
set(ICAR_PROJECT_NAME "paddle_detection")
set(ICAR_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/paddle_detection.cpp)
add_executable(${ICAR_PROJECT_NAME} ${ICAR_PROJECT_SOURCES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${OpenCV_LIBS})
target_link_libraries(${ICAR_PROJECT_NAME}  pthread )

# 多线程测试
set(ICAR_PROJECT_NAME "test")
set(ICAR_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/test.cpp)
add_executable(${ICAR_PROJECT_NAME} ${ICAR_PROJECT_SOURCES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${OpenCV_LIBS})
target_link_libraries(${ICAR_PROJECT_NAME}  pthread )

# pid调试
set(ICAR_PROJECT_NAME "uart_test")
set(ICAR_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/uart_test.cpp)
add_executable(${ICAR_PROJECT_NAME} ${ICAR_PROJECT_SOURCES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${SERIAL_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${OpenCV_LIBS})

# 断路区域调试
set(ICAR_PROJECT_NAME "cone_blue_enrode")
set(ICAR_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/cone_blue_enrode.cpp)
add_executable(${ICAR_PROJECT_NAME} ${ICAR_PROJECT_SOURCES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${OpenCV_LIBS})

#---------------------------------------------------------------------
#               [ bin ] ==> [ main ]
#---------------------------------------------------------------------
set(ICAR_PROJECT_NAME "icar")
set(ICAR_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/src/icar.cpp)
add_executable(${ICAR_PROJECT_NAME} ${ICAR_PROJECT_SOURCES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${OpenCV_LIBS})
target_link_libraries(${ICAR_PROJECT_NAME}  pthread )
target_link_libraries(${ICAR_PROJECT_NAME}  ${SERIAL_LIBRARIES})

