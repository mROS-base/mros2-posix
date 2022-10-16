# mros2-posix

mROS 2 (formly `mros2`) realizes a agent-less and lightweight runtime environment compatible with ROS 2 for embedded devices. mROS 2 mainly offers pub/sub APIs compatible with [rclcpp](https://docs.ros2.org/dashing/api/rclcpp/index.html) for embedded devices.

mROS 2 consists of communication library for pub/sub APIs, RTPS protocol, UDP/IP stack, and real-time kernel. This repository provides the reference implementation(`mros2-posix`) of mROS 2 that can be operated on the `Linux process`. Please also check [mros2 repository](https://github.com/mROS-base/mros2) for more details and another implementations.

## Supported environment

* ROS 2 Host environment for communication:
  * [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04 LTS
  * [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/index.html) on Ubuntu 20.04 LTS
* Host environment for building mros2-posix app:
  * Ubuntu 20.04 LTS
  * Ubuntu 18.04 LTS
* mros2-posix execution environment:
  * Ubuntu 20.04 LTS
  * Ubuntu 18.04 LTS
  * The followings can be also used but not fully tested
	  * WSL1 on Windows 10
	  * docker compose
  * The embedded boards and RTOSes are not supported yet,,,

## Envorinmental setup

### ROS 2 Host environment

Please refer to the public documentation

* [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html) on Ubuntu 22.04 LTs
* [Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html) on Ubuntu 20.04 LTs

### Host environment for building mros2-posix app:

Install following tools for building mros2-posix application.

```
sudo apt-get update && sudo apt-get install -y \
  git wget \
  build-essential gcc g++ \
  libssl-dev libreadline-dev zlib1g-dev \
  make autoconf automake cmake \
  pkg-config curl \
  net-tools netcat
```

### mros2-posix execution environment

Please check the IP address and netmask of the execution environment. 

Example (ethernet port `enp4s0` in this case):

```
$ ifconfig
<sniped.>
enp4s0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.11.3  netmask 255.255.255.0  broadcast 192.168.11.255
        ether cc:30:80:3a:7a:de  txqueuelen 1000  (Ethernet)
        RX packets 1292708  bytes 964243524 (964.2 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 920815  bytes 124650942 (124.6 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
<sniped.>
```

Here, IP address and netmask are `192.168.11.3` and `255.255.255.0`, respectively.

You need to edit the below files to set IP address and netmask.

* IP address and netmask to `workspace/include/netif.h`
* IP address to `workspace/include/rtps/config.h`

## Quickstart

This section explains how to build and execute mros2-posix application as a Linux/POSIX process, using `pub_string` as an example (please see workspace/README.md for another examples).


### Build for mros2-posix app

First of all, clone this repository. Note that **--recursive** is mandatory.

```
git clone --recursive https://github.com/mROS-base/mros2-posix.git
```

Please set your IP address to `IP_ADDRESS` in `workspace/include/rtps/config.h`, and IP address and netmask to `NETIF_IPADDR` and `NETIF_NETMASK` in `workspace/include/netif.h`.

Move to mros2-posix and build with the target app name.

```
cd mros2-posix/
bash build.bash clean
bash build.bash all pub_string
```

Once build process can be completed, you can find `mros2-posix` executable in `cmake_build/`. 

### Run the example

#### mros2-posix environment

Run the mros2-posix with your IP address and netmask.

```
./cmake_build/mros2-posix <IPaddr> <netmask>
```

Example:

```
$ ./cmake_build/mros2-posix 192.168.11.3 255.255.255.0
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
source /opt/ros/humble/setup.bash   # or, source /opt/ros/foxy/setup.bash
ros2 topic echo /to_stm
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

Please see [workspace](https://github.com/mROS-base/mros2-posix/tree/main/workspace) for example applications.

## Submodules and Licenses

The source code of this repository itself is published under Apache License 2.0.
Please note that this repository contains the following stacks as the submodules, and also check their Licenses.

* [mros2](https://github.com/mROS-base/mros2): the pub/sub APIs compatible with ROS 2 Rclcpp
  * [embeddedRTPS](https://github.com/mROS-base/embeddedRTPS): RTPS communication layer (including lwIP and Micro-CDR)
* [cmsis-posix](https://github.com/mROS-base/cmsis-posix): Interface layer between CMSIS OS API and POSIX compiliant kernel (e.g., Linux)
* [lwip-posix](https://github.com/mROS-base/lwip-posix): LwIP UDP/IP implementation based on POSIX compliant kernel (e.g., Linux)
