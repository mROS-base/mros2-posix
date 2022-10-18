set(APL_CMAKE_VERSION 3.5.1)
cmake_minimum_required(VERSION ${APL_CMAKE_VERSION})

project(mros2-posix-project
  VERSION 1.0.0
  LANGUAGES CXX C
)

set(debug true)
set(CMAKE_C_FLAGS "-std=gnu99")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wunknown-pragmas")
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wtrigraphs")
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wimplicit-int")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

add_compile_definitions(OS_POSIX)
add_compile_definitions(osObjectsExternal)
add_compile_definitions(STM32F767xx)

message(STATUS "CMAKE_APPNAME=${CMAKE_APPNAME}")
set(MROS2_APPNAME ${CMAKE_APPNAME})

# CMSIS lib
add_subdirectory(cmsis-posix)

set(cmsis_DIR "${PROJECT_SOURCE_DIR}/cmsis-posix/public")
message(STATUS "cmsis_DIR=${cmsis_DIR}")
find_package(cmsis REQUIRED)

# LWIP lib
add_subdirectory(lwip-posix)

set(lwip_DIR "${PROJECT_SOURCE_DIR}/lwip-posix/public")
message(STATUS "lwip_DIR=${lwip_DIR}")
find_package(lwip REQUIRED)

# mros2 & embeddedRTPS lib
set(mros2_DIR "${PROJECT_SOURCE_DIR}/mros2")
message(STATUS "mros2_DIR=${mros2_DIR}")
add_library(mros2 INTERFACE)

set(CMAKE_OS_POSIX true)
add_subdirectory(mros2)

# Application combines (CMSIS LWIP EmbeddedRTPS mROS2)
include(workspace/${MROS2_APPNAME}/Filelists.cmake) 
add_executable(mros2-posix
  ${apl_SRCS}
)

target_include_directories(
  mros2-posix

  # CMSIS INCLUDES
  PRIVATE ${cmsis_DIR}/include

  # LWIP INCLUDES
  PRIVATE ${lwip_DIR}/include/posix
  PRIVATE ${lwip_DIR}/include/lwip
  PRIVATE ${lwip_DIR}/include/system

  # embeddedRTPS INCLUDES
  PRIVATE ${mros2_DIR}/include

  # mROS2 INCLUDES
  PRIVATE ${mros2_DIR}/include/mros2
  PRIVATE ${mros2_DIR}/include/mros2_msgs

  # rtps/config.h
  PRIVATE ${PROJECT_SOURCE_DIR}/include
  # app
  PRIVATE ${PROJECT_SOURCE_DIR}/workspace/${MROS2_APPNAME}
  PRIVATE ${PROJECT_SOURCE_DIR}/workspace/custom_msgs
)

target_link_libraries(mros2-posix mros2 lwip cmsis pthread)
