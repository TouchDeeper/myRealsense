#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealSensePCLExample)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
include(../CMake/opengl_config.cmake)
#PCL
find_package(PCL REQUIRED)
include_directories(rs-pcl ${PCL_INCLUDE_DIRS}  ../third-party/imgui)
link_directories(${PCL_LIBRARY_DIRS})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

add_executable(rs-pcl rs-pcl.cpp ../example.hpp ../third-party/imgui/imgui.cpp ../third-party/imgui/imgui_draw.cpp ../third-party/imgui/imgui_impl_glfw.cpp)
target_link_libraries(rs-pcl ${DEPENDENCIES} ${PCL_LIBRARIES})
