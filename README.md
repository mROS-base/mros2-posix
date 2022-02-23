# mros2-posix

mROS 2 (formly `mros2`) realizes a agent-less and lightweight runtime environment compatible with ROS 2 for embedded devices. mROS 2 mainly offers pub/sub APIs compatible with [rclcpp](https://docs.ros2.org/dashing/api/rclcpp/index.html) for embedded devices.

mROS 2 consists of communication library for pub/sub APIs, RTPS protocol, UDP/IP stack, and real-time kernel. This repository provides the reference implementation(`mros2-posix`) of mROS 2 that can be operated on the `Linux process`. Please also check [mros2 repository](https://github.com/mROS-base/mros2) for more details and another implementations.

## Supported environment

* ROS 2 version
  * [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/index.html)
* ROS 2 host environment
  * Ubuntu 20.04 LTS
* mROS 2 host environment
  * WSL1 on Windows 10
  * Linux(TODO)
  * docker
    * docker compose on Docker Desktop
    * docker compose on Linux(TODO)
* mROS 2 Build and execution OS
  * Ubuntu 20.04 LTS
  * Ubuntu 18.04 LTS

# Envorinmental setup

### ROS 2 host environment
Install [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html) on the ROS 2 host environment.

### mROS 2 host environment
1. Install build tools on the mROS 2 host environment.
```
$ apt-get update && apt-get install -y \
	git	\
	build-essential	\
	wget	\
	gcc	\
	g++	\
	vim	\
	libssl-dev libreadline-dev zlib1g-dev \
	make	\
	autoconf \
	automake \
	pkg-config \
	curl \
	net-tools \
	netcat \
	cmake \
	&& apt-get clean
$ apt update -y
$ apt upgrade -y
```
2. Please check the `IP address` and `netmask` of the host environment's ethernet connecting on your LAN.

Example:

```
$ ifconfig
  :
eth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.11.49  netmask 255.255.255.0  broadcast 192.168.11.255
        inet6 fe80::8877:2824:2cca:6ac6  prefixlen 64  scopeid 0xfd<compat,link,site,host>
  :
```

`IP address` and `netmask` will be used on the following cases.

* Setting `IP address` of embeddedRTPS config.h
* Setting `IP address`  and `netmask` as arguments of mros-posix application


## Quickstart
This section explains how to build and execute mROS 2 with Linux process, using `pub_string` application as an example.

### Build for mROS 2 app

First of all, clone this repository. Note that **--recursive** is mandatory.

```
$ git clone --recursive git@github.com:mROS-base/mros2-posix.git
```

Please set your `IP address` on `IP_ADDRESS` in [mros2-posix/workspace/include/rtps/config.h](https://github.com/mROS-base/mros2-posix/blob/main/workspace/include/rtps/config.h).

Move to mros2-posix and build with the target app name (please see workspace/README.md for another examples).

```
$ cd mros2-posix
$ bash build.bash pub_string clean
$ bash build.bash pub_string all
```

Once build process can be completed, you can find mros2-posix as the binary on cmake-build. 

```
$ ls cmake-build/mros2-posix
cmake-build/mros2-posix
```

### Run the example

### mROS 2 host environment
Run the mros2-posix with your host `IP address` and `netmask`.

```
$ ./cmake-build/mros2-posix <IP address> <netmask>
```

Example:

```
$ ./cmake-build/mros2-posix 192.168.11.49 255.255.255.0
  :
LOG_NOTICE : 00000000.128 : thread_udp_recv:UP: mcp=0x7fb1c0000e20
LOG_DEBUG : 00000000.129 : [MROS2LIB] successfully created participant
LOG_DEBUG : 00000000.129 : mROS 2 initialization is completed
LOG_DEBUG : 00000000.130 : [MROS2LIB] create_publisher complete.
LOG_NOTICE : 00000000.130 : ready to publish message
LOG_NOTICE : 00000000.130 : publishing msg: 'Hello from mROS 2!! 0'
LOG_DEBUG : 00000000.131 : [MROS2LIB] Initilizing Domain complete
LOG_NOTICE : 00000001.151 : publishing msg: 'Hello from mROS 2!! 1'
LOG_NOTICE : 00000002.162 : publishing msg: 'Hello from mROS 2!! 2'
LOG_NOTICE : 00000003.171 : publishing msg: 'Hello from mROS 2!! 3'
LOG_NOTICE : 00000004.182 : publishing msg: 'Hello from mROS 2!! 4'
LOG_NOTICE : 00000005.186 : publishing msg: 'Hello from mROS 2!! 5'
LOG_NOTICE : 00000006.198 : publishing msg: 'Hello from mROS 2!! 6'
```


### ROS 2 host environment
Launch ROS 2 topic echo node.

```
$ source /opt/ros/foxy/setup.bash
$ ros2 topic echo /to_stm
```

Now, you can confirm the message.
```
data: Hello from mROS 2!! 4
---
data: Hello from mROS 2!! 5
---
data: Hello from mROS 2!! 6
---
data: Hello from mROS 2!! 7
---
data: Hello from mROS 2!! 8
---
```

## Example applications
Please see [workspace](https://github.com/mROS-base/mros2-posix/tree/main/workspace/src) for example applications.

## Submodules and Licenses
The source code of this repository itself is published under Apache License 2.0.
Please note that this repository contains the following stacks as the submodules, and also check their Licenses.

* [mros2](https://github.com/mROS-base/mros2): the pub/sub APIs compatible with ROS 2 Rclcpp
  * [embeddedRTPS](https://github.com/mROS-base/embeddedRTPS): RTPS communication layer (including lwIP and Micro-CDR)
* [cmsis-posix](https://github.com/mROS-base/cmsis-posix): Interface layer between CMSIS OS API and Posix based OS(Linux)
* [lwip-posix](https://github.com/mROS-base/lwip-posix): LwIP UDP/IP implementation based on Posix OS(Linux)
