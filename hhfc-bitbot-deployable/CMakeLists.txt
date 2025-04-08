cmake_minimum_required(VERSION 3.14)

project(bitbot)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_BUILD_TYPE DEBUG)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(BITBOT_DEPENDENCY_USE_PROXY TRUE)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_SOURCE_DIR}/dependencies)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/huahui/Library/libtorch)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/huahui/Library/cifx)

set(TORCH_PATH ~/Library/libtorch)
include_directories(${TORCH_PATH}/include)
include_directories(${TORCH_PATH}/include/torch/csrc/api/include)

message("${TORCH_INCLUDE_DIRS}")
include_directories("/home/huahui/Library/cifx/include/")
link_directories("/home/huahui/Library/cifx/lib/")

include(bitbot.cmake)

BitbotAddDependency(
  NAME           bitbot_cifx
  GIT_REPO       git@github.com:limymy/bitbot_cifx.git
  GIT_TAG        main
  FILE_NAME      "bitbot_cifx_zdown_imu.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
)

IF(CMAKE_BUILD_TYPE STREQUAL Debug)
  message(STATUS "build in debug mode")
  add_definitions(-DDEBUG)
ENDIF()

set(EXECUTABLE_OUTPUT_PATH
    ${CMAKE_BINARY_DIR}/bin
    ${EIGEN_SRC}
    )

    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)
message(STATUS "Torch version=${Torch_VERSION}")

add_subdirectory(./inekf-warper)

add_executable(
  main_app 
  main.cpp
  user_func.h
  user_func.cpp 
  leeMatrix.c
  inference_net.hpp
  ZenBuffer.hpp
  )

target_include_directories(main_app PUBLIC
  ${TORCH_INCLUDE_DIRS}
  3rd_party/ViconSDK
)

target_link_libraries(main_app
  BitbotKernel
  BitbotCifx
  ${TORCH_LIBRARIES}
  ViconDataStreamSDK_CPP
  inekf
)
target_link_directories(main_app PUBLIC 3rd_party/ViconSDK/lib)
target_compile_features(main_app PUBLIC cxx_std_20)
