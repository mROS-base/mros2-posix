#!/bin/bash

if [ $# -gt 2 ] || [ $# -eq 0 ]
then
  echo "Usage: $0 {all|up|clean} appname"
  exit 1
fi

OPT=${1}
APPNAME=${2}

function clean_subdirectory()
{
  rm -rf ${1}/cmake_build
  rm -rf ${1}/public
}

if [ ${OPT} == "clean" ];
then
  echo "build operation is set to clean"
  rm -rf ./cmsis-posix/Third_Party/FreeRTOS
  rm -rf ./lwip-posix/Third_Party/STM32CubeF7
  clean_subdirectory cmsis-posix
  clean_subdirectory lwip-posix
  #clean_subdirectory mros2
  rm -rf cmake_build/
  echo "build clean is completed"
  exit 0
fi

if [ $# -ne 2 ]
then
  echo "Usage: $0 {all|up|clean} appname"
  exit 1
fi

if [ -d workspace/${APPNAME} ]
then
  :
else
  echo "ERROR: can not find appname=${APPNAME} on workspace/"
  exit 1
fi


if [ -d cmake_build ]
then
  :
else
  mkdir cmake_build
fi

function download_files()
{
  local dir=${1}
  cd ${dir}
  bash Third_Party/download.bash
  cd ..
}

function build_subdirectory()
{
  local dir=${1}/cmake_build
  if [ -d ${dir} ]
  then
    :
  else
    mkdir -p ${dir}
  fi
  cd ${dir}
  if [ $# -eq 1 ]
  then
    cmake ..
  else
    cmake .. -D ${2}
  fi
  make
  if [ -d ../public/include ]
  then
    :
  else
    make install
  fi
  cd ../..
}

# generate of header file for template functions of MsgType
function generate_template_functions()
{
	MROS2DIR=../mros2
	TEMPLATESGEN_FILE=${MROS2DIR}/mros2_header_generator/templates_generator.py

	echo "INFO: generate header file for template functions of MsgType"
	cd workspace
	python3 ${TEMPLATESGEN_FILE} ${APPNAME}
	if [ $? -eq 0 ];
	then
	  echo "INFO: header fille for template function of ${APPNAME}'s MsgType is successfully generated"
	else
	  echo "ERROR: failed to generate header fille for template function of ${APPNAME}'s MsgType"
	  exit 1
	fi
	cd ..
}

if [ ${OPT} = "all" ]
then
  download_files cmsis-posix
  build_subdirectory cmsis-posix 
  download_files lwip-posix 
  build_subdirectory lwip-posix 
  generate_template_functions
  #build_subdirectory mros2 CMAKE_OS_POSIX=true
  cd cmake_build
  cmake .. -D CMAKE_APPNAME=${APPNAME}
  make
  cd ..
elif [ ${OPT} = "up" ]
then
  cd cmake_build
  cmake .. -D CMAKE_APPNAME=${APPNAME}
  make
  cd ..
else
  echo "ERROR: wrong operation ${OPT}"
  exit 1
fi
