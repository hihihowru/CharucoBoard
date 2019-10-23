cmake_minimum_required(VERSION 3.1.0)

project(charuco_pose_estimator)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL 1.2 REQUIRED)

# find_package(opencv REQUIRED COMPONENTS core opencv rgbd highgui videoio )
find_package(OpenCV PATHS /usr/local/include/opencv4)
find_package(realsense2 PATHS /usr/local/include/librealsense2)
#set cmake export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS 14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#endif()
#find package realsense2
find_package(realsense2 REQUIRED )
find_package(OpenCV REQUIRED )

# Simple non robust way to find the librealsense library
if(WIN32)
    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    else()
        set(LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x86")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x86")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    endif()
    set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/\$\(Configuration\)")                # TODO: Update this variable to correct path - folder to which your project will compile
    set(ADDITIONAL_INCLUDE_DIRS "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\include")   # TODO: Update this variable to correct path - folder where librealsense2 folder is found
endif()


find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(main main.cpp)
include_directories(main ${ADDITIONAL_INCLUDE_DIRS})
include_directories(main ${OpenCV_INCLUDE_DIRS})
target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries(main ${REALSENSE2_FOUND})
target_link_libraries (main ${PCL_LIBRARIES})
target_link_libraries(main usb-1.0)


# find_package( REQUIRED)
set(DEPENDENCIES realsense2 ${OpenCV_LIBS})

