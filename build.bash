#!/bin/bash

if [ $# -ne 1 -a $# -ne 2 ]
then
	echo "Usage: $0 {all|up|clean} [appname]"
	echo "appname:"
	for i in `ls workspace/src`
	do
		echo "  $i"
	done
	exit 1
fi

OPT=${1}

if [ $# -eq 2 ]
then
	APPNAME=${2}
	cat CMakeLists.tpl | sed 's/template-appname/'${APPNAME}'/g' > CMakeLists.txt
else
	if [ -f CMakeLists.txt ]
	then
		:
	else
		APPNAME=pub_string
		cat CMakeLists.tpl | sed 's/template-appname/'${APPNAME}'/g' > CMakeLists.txt

	fi
fi

if [ -d workspace/src/${APPNAME} ]
then
	:
else
	echo "ERROR: can not find appname=${APPNAME} on workspace/src"
	exit 1
fi


if [ -d cmake-build ]
then
	:
else
	mkdir cmake-build
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
	local dir=${1}/cmake-build
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
		cmake .. -D ${2} -D ${3}
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
function clean_subdirectory()
{
	rm -rf ${1}/cmake-build/CMakeFiles
	rm -f ${1}/cmake-build/cmake_install.cmake
	rm -f ${1}/cmake-build/CMakeCache.txt
	rm -f ${1}/cmake-build/install_manifest.txt
	rm -f ${1}/cmake-build/*.a
	rm -f ${1}/cmake-build/Makefile
	rm -rf ${1}/public/*
}

if [ ${OPT} = "all" ]
then
	download_files cmsis-posix
	build_subdirectory cmsis-posix 
	download_files lwip-posix 
	build_subdirectory lwip-posix 
	build_subdirectory mros2 CMAKE_OS_POSIX=true RTPS_CONFIG_INCLUDE_DIR=`pwd`/workspace/include
	cd cmake-build
	cmake ..
	make
	cd ..
elif [ ${OPT} = "up" ]
then
	cd cmake-build
	cmake ..
	make
	cd ..
else
	rm -rf ./lwip-posix/Third_Party/STM32CubeF7
	clean_subdirectory cmsis-posix 
	clean_subdirectory lwip-posix 
	clean_subdirectory mros2
	cd cmake-build
	rm -rf ./*
	cd ..
fi

