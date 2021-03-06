#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(rs-callback)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_BUILD_TYPE "Release")
#change here
include(../CMake/opengl_config.cmake)
find_package(Threads REQUIRED)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES} Threads::Threads)
include_directories(${PROJECT_NAME} ../ ../third-party/tclap/include)
