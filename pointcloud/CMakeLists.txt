#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesPointcloud)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
#change here
include(../CMake/opengl_config.cmake)
include_directories(rs-pointcloud ../)
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
#PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

#TdLib
set(TdLib_DIR "/home/wang/CLionProjects/TdLibrary/install/lib/CMake/TdLib/")
find_package(TdLib)
include_directories(${TdLib_INCLUDE_DIRS})

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

#pointcloud
add_executable(rs-pointcloud rs-pointcloud.cpp ../example.hpp)
target_link_libraries(rs-pointcloud
                        ${DEPENDENCIES}
                        Threads::Threads
                        ${OpenCV_LIBRARIES}
                        ${PCL_LIBRARIES}
                        ${TdLib_LIBRARIES})