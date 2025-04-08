# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.5)
# Project's name
project(zzs_inekf_warper)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message("use hot reload")
	set(CMAKE_CXX_FLAGS_DEBUG "/MDd /ZI /Ob0 /Od /RTC1")
endif()

option(USE_MUTEX "Use mutex in code" OFF)
option(BUILD_EXAMPLE "if build contact aid example" OFF)

message("USE_MUTEX: " ${USE_MUTEX})
message("BUILD_EXAMPLE:" ${BUILD_EXAMPLE})

if (USE_MUTEX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DINEKF_USE_MUTEX=true")
else (USE_MUTEX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DINEKF_USE_MUTEX=false")
endif(USE_MUTEX)

message("CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})
message("project inekf current build dir ${PROJECT_SOURCE_DIR}")

find_package(Eigen3 REQUIRED)
# find_package(pugixml CONFIG REQUIRED)


# Create Library
add_library(inekf SHARED
    "${PROJECT_SOURCE_DIR}/invariant-ekf/InEKF.cpp"
    "${PROJECT_SOURCE_DIR}/invariant-ekf/RobotState.cpp"
    "${PROJECT_SOURCE_DIR}/invariant-ekf/LieGroup.cpp"
    "${PROJECT_SOURCE_DIR}/invariant-ekf/NoiseParams.cpp"
    "${PROJECT_SOURCE_DIR}/invariant-ekf/libdef.h"
    "${PROJECT_SOURCE_DIR}/InEKF_Warper.hpp"
    "${PROJECT_SOURCE_DIR}/ContactEstimator.hpp"
    "${PROJECT_SOURCE_DIR}/ConfigParser.hpp"
)

if(WIN32)
    message("Build on Windows")
    target_compile_definitions(inekf PRIVATE _OS_WINDOWS_)
endif()

target_compile_definitions(inekf PRIVATE export_inekf)

# pugixml::pugixml
target_link_libraries(inekf PUBLIC Eigen3::Eigen )

target_include_directories(inekf PUBLIC
    "${PROJECT_SOURCE_DIR}/invariant-ekf"
    "${PROJECT_SOURCE_DIR}"
)

######################build example######################
if (BUILD_EXAMPLE)
    add_executable(example 
    "${PROJECT_SOURCE_DIR}/example/csv_io.hpp"
    "${PROJECT_SOURCE_DIR}/example/main.cpp"
    )
    target_link_libraries(example PRIVATE inekf)
endif(BUILD_EXAMPLE)
