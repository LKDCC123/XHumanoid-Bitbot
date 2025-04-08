get_property(
  BITBOT_CMAKE_MODULE_INITIALIZED GLOBAL ""
  PROPERTY BITBOT_CMAKE_MODULE_INITIALIZED
  SET
)
if(BITBOT_CMAKE_MODULE_INITIALIZED)
  return()
endif()

set_property(GLOBAL PROPERTY BITBOT_CMAKE_MODULE_INITIALIZED true)

#######################################################################################
### GET CPM START

set(CPM_DOWNLOAD_VERSION 0.38.6)

if(CPM_SOURCE_CACHE)
  set(CPM_DOWNLOAD_LOCATION "${CPM_SOURCE_CACHE}/cpm/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
elseif(DEFINED ENV{CPM_SOURCE_CACHE})
  set(CPM_DOWNLOAD_LOCATION "$ENV{CPM_SOURCE_CACHE}/cpm/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
else()
  set(CPM_DOWNLOAD_LOCATION "${CMAKE_BINARY_DIR}/cmake/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
endif()

if(BITBOT_DEPENDENCY_USE_LOCAL_FILE)
  set(CPM_DOWNLOAD_LOCATION "${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
endif()

# Expand relative path. This is important if the provided path contains a tilde (~)
get_filename_component(CPM_DOWNLOAD_LOCATION ${CPM_DOWNLOAD_LOCATION} ABSOLUTE)

function(download_cpm)
  message(STATUS "Downloading CPM.cmake to ${CPM_DOWNLOAD_LOCATION}")
  if(BITBOT_DEPENDENCY_USE_PROXY)
    file(DOWNLOAD
        https://ghproxy.com/github.com/cpm-cmake/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake
        ${CPM_DOWNLOAD_LOCATION}
    )
  else()
    file(DOWNLOAD
      https://github.com/cpm-cmake/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake
      ${CPM_DOWNLOAD_LOCATION}
    )
  endif()
endfunction()

if(NOT (EXISTS ${CPM_DOWNLOAD_LOCATION}))
  download_cpm()
else()
  # resume download if it previously failed
  file(READ ${CPM_DOWNLOAD_LOCATION} check)
  if("${check}" STREQUAL "")
    download_cpm()
  endif()
endif()

include(${CPM_DOWNLOAD_LOCATION})

### GET CPM END
#######################################################################################

function(BitbotProxyUrl input_url output_url)
  string(FIND ${input_url} "http://" USE_HTTP)
  string(FIND ${input_url} "https://" USE_HTTPS)
  if(NOT ${USE_HTTP} EQUAL "-1")
    string(REPLACE "http://" "http://ghproxy.com/" input_url ${input_url})
  elseif(NOT ${USE_HTTPS} EQUAL "-1")
    string(REPLACE "https://" "https://ghproxy.com/" input_url ${input_url})
  endif()
  set(${output_url} ${input_url} PARENT_SCOPE)
endfunction(BitbotProxyUrl)

macro(BitbotAddDependency)
  set(optionsArgs "")
  set(oneValueArgs
    NAME
    GIT_REPO
    GIT_TAG
    GIT_SHALLOW
    URL
    FILE_NAME
    FILE_PATH
    USE_PROXY
    USE_LOCAL_FILE
  )
  set(multiValueArgs OPTIONS)
  cmake_parse_arguments(BIT_ARGS "${optionsArgs}" "${oneValueArgs}" "${multiValueArgs}" "${ARGN}")

  if(NOT DEFINED BIT_ARGS_USE_PROXY)
    set(BIT_ARGS_USE_PROXY FALSE)
  endif()
  if(NOT DEFINED BIT_ARGS_USE_LOCAL_FILE)
    set(BIT_ARGS_USE_LOCAL_FILE FALSE)
  endif()

  if(${BIT_ARGS_USE_LOCAL_FILE})
    string(PREPEND BIT_ARGS_FILE_NAME "${BIT_ARGS_FILE_PATH}/")
    message(STATUS "Bitbot dependency:${BIT_ARGS_NAME} local_file:${BIT_ARGS_FILE_NAME}")
    CPMAddPackage(
      NAME          ${BIT_ARGS_NAME}
      URL           ${BIT_ARGS_FILE_NAME}
      OPTIONS       ${BIT_ARGS_OPTIONS}
    )
  elseif(DEFINED BIT_ARGS_URL)
    set(USE_URL TRUE)
    if(${BIT_ARGS_USE_PROXY})
      BitbotProxyUrl(${BIT_ARGS_URL} BIT_ARGS_URL)
    endif()
    message(STATUS "Bitbot dependency:${BIT_ARGS_NAME} URL:${BIT_ARGS_URL}")
    CPMAddPackage(
      NAME          ${BIT_ARGS_NAME}
      URL           ${BIT_ARGS_URL}
      OPTIONS       ${BIT_ARGS_OPTIONS}
    )
  elseif(DEFINED BIT_ARGS_GIT_REPO)
    if(${BIT_ARGS_USE_PROXY})
      BitbotProxyUrl(${BIT_ARGS_GIT_REPO} BIT_ARGS_GIT_REPO)
    endif()
    message(STATUS "Bitbot dependency:${BIT_ARGS_NAME} GIT_REPO:${BIT_ARGS_GIT_REPO}")
    CPMAddPackage(
      NAME           ${BIT_ARGS_NAME}
      GIT_REPOSITORY ${BIT_ARGS_GIT_REPO}
      GIT_TAG        ${BIT_ARGS_GIT_TAG}
      GIT_SHALLOW    ${BIT_ARGS_GIT_SHALLOW}
      OPTIONS        ${BIT_ARGS_OPTIONS}
    )
  endif()
endmacro(BitbotAddDependency)
