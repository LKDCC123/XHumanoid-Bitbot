cmake_minimum_required(VERSION 3.14)

project(bitbot)

set(BITBOT_DEPENDENCY_USE_PROXY OFF)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_SOURCE_DIR}/dependencies)

include(bitbot.cmake)

BitbotAddDependency(
  NAME           bitbot_mujoco
  FILE_NAME      "bitbot_mujoco-main.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_LOCAL_FILE TRUE
)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

add_executable(main_app main.cpp user_func.cpp)
target_link_libraries(main_app
  BitbotMujoco
)
target_compile_options(main_app PUBLIC)
target_compile_features(main_app PUBLIC cxx_std_20)
