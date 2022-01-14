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

#set(MROS2_APPNAME "pub_string")
#set(MROS2_APPNAME "sub_string")
#set(MROS2_APPNAME "udp_multicast")
set(MROS2_APPNAME "fastdds-comp")

# CMSIS lib
add_subdirectory(cmsis-posix)

# LWIP lib
add_subdirectory(lwip-posix)

# mros2 & EmbeddedRTPS lib
set(CMAKE_OS_POSIX true)
set(RTPS_CONFIG_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/workspace/include")
add_subdirectory(mros2)

set(cmsis_DIR "${PROJECT_SOURCE_DIR}/cmsis-posix/public")
message(STATUS "cmsis_DIR=${cmsis_DIR}")
find_package(cmsis REQUIRED)

set(lwip_DIR "${PROJECT_SOURCE_DIR}/lwip-posix/public")
message(STATUS "lwip_DIR=${lwip_DIR}")
find_package(lwip REQUIRED)

set(mros2_DIR "${PROJECT_SOURCE_DIR}/mros2/public")
message(STATUS "mros2_DIR=${mros2_DIR}")
find_package(mros2 REQUIRED)

# Application combines (CMSIS LWIP EmbeddedRTPS mROS2)
include(workspace/src/${MROS2_APPNAME}/Filelists.cmake) 
add_executable(mros2-posix 
	${apl_SRCS}
)

target_include_directories(
	mros2-posix 

  #CMSIS INCLUDES
  PRIVATE ${cmsis_DIR}/include

  #LWIP INCLUDES
  PRIVATE ${lwip_DIR}/include/posix
  PRIVATE ${lwip_DIR}/include/lwip
  PRIVATE ${lwip_DIR}/include/system

  #EmbeddedRTPS INCLUDES
  PRIVATE ${mros2_DIR}/include

  #mROS2 INCLUDES
  PRIVATE ${mros2_DIR}/include/mros2
  PRIVATE ${mros2_DIR}/include/mros2_msgs

  #app
  PRIVATE ${RTPS_CONFIG_INCLUDE_DIR}
  PRIVATE ${PROJECT_SOURCE_DIR}/workspace/src/${MROS2_APPNAME}
)
target_link_libraries(mros2-posix mros2 lwip cmsis pthread)
