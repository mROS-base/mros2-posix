# mros2-posix

mROS 2 (formly `mros2`) realizes a agent-less and lightweight runtime environment compatible with ROS 2 for embedded devices. mROS 2 mainly offers pub/sub APIs compatible with [rclcpp](https://docs.ros2.org/dashing/api/rclcpp/index.html) for embedded devices. mROS 2 consists of communication library for pub/sub APIs, RTPS protocol, UDP/IP stack, and real-time kernel. 

This repository provides the implementation of `mros2` as the **pthread**. In other words, `mros2` nodes can be operated onto native Linux kernel and general-purpose computers (e.g., Ubuntu onto x64_PC or arm64_RPi4). We also provide the POSIX layer (`cmsis-posix` and `lwip-posix`) that are prepared for mROS 2 and embeddedRTPS .
Please also check [mros2 repository](https://github.com/mROS-base/mros2) for more details and another implementations onto embedded devices.

## Supported environment

* ROS 2 environment as the communication partner (IOW, host environment):
  * [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04 LTS
  * [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/index.html) on Ubuntu 20.04 LTS
* `mros2-posix` environment for building & executing app:
  * Ubuntu 22.04 LTS
  * Ubuntu 20.04 LTS
  * The followings can be also used but not fully tested
	  * WSL1 on Windows 10
	  * docker compose

## Envorinmental setup

### ROS 2 Host environment

Please refer to the public documentation

* [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html) on Ubuntu 22.04 LTs
* [Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html) on Ubuntu 20.04 LTs

### `mros2-posix` environment for building & executing app:

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

Here, IP address and netmask are `192.168.11.3` and `255.255.255.0`, respectively in this example.

You need to edit the below files to set IP address and netmask.

* IP address and netmask to `include/netif.h`
* IP address to `include/rtps/config.h`

## Getting Started

This section explains how to build and execute mros2-posix application as a Linux/POSIX process, using `echoback_string` as an example (please see workspace/README.md for another examples).

### Build for mros2-posix app

First of all, clone this repository. Note that **--recursive** is mandatory.

```
git clone --recursive https://github.com/mROS-base/mros2-posix.git
```

Please set your network information to the below file.

- IP address to `IP_ADDRESS` in `include/rtps/config.h`
- IP address and netmask to `NETIF_IPADDR` and `NETIF_NETMASK` in `include/netif.h`.

Move to `mros2-posix/` and build with the target app name.

```
cd mros2-posix/
bash build.bash clean
bash build.bash all echoback_string
```

Once build process is successfully completed, you can find `mros2-posix` executable in `cmake_build/`. 

### Run the example

Run the mros2-posix application.

```
./cmake_build/mros2-posix
```

Example log:

```
$ ./cmake_build/mros2-posix
...(SNIPPED)...
LOG_NOTICE : 00000000.101 : thread_udp_recv:UP: mcp=0x7f4e40000e00
LOG_DEBUG : 00000000.101 : [MROS2LIB] successfully created participant
LOG_DEBUG : 00000000.101 : [MROS2LIB] create_publisher complete.
LOG_DEBUG : 00000000.101 : [MROS2LIB] create_subscription complete.
LOG_NOTICE : 00000000.201 : ready to pub/sub message

publishing msg: 'Hello from mros2-posix onto Linux: 0'
LOG_DEBUG : 00000000.201 : [MROS2LIB] Initilizing Domain complete
publishing msg: 'Hello from mros2-posix onto Linux: 1'
publishing msg: 'Hello from mros2-posix onto Linux: 2'
publishing msg: 'Hello from mros2-posix onto Linux: 3'
publishing msg: 'Hello from mros2-posix onto Linux: 4'
publishing msg: 'Hello from mros2-posix onto Linux: 5'
publishing msg: 'Hello from mros2-posix onto Linux: 6'
publishing msg: 'Hello from mros2-posix onto Linux: 7'
publishing msg: 'Hello from mros2-posix onto Linux: 8'
...(SNIPPED)...
```

### Run native ROS 2 node on the host environment

One of the easiest way to operate the host is using Docker. On the host terminal, type the command below.

```
docker run --rm -it --net=host ros:humble /bin/bash \
  -c "source /opt/ros/humble/setup.bash &&
  cd &&
  git clone https://github.com/mROS-base/mros2-host-examples &&
  cd mros2-host-examples &&
  colcon build --packages-select mros2_echoreply_string &&
  source install/setup.bash &&
  ros2 run mros2_echoreply_string echoreply_node"
```

Then, we can confirm the communication between the PC and mros2-posix process via ROS 2.

```
Cloning into 'mros2-host-examples'...
remote: Enumerating objects: 831, done.
remote: Counting objects: 100% (85/85), done.
remote: Compressing objects: 100% (68/68), done.
remote: Total 831 (delta 46), reused 26 (delta 15), pack-reused 746
Receiving objects: 100% (831/831), 96.01 KiB | 7.38 MiB/s, done.
Resolving deltas: 100% (448/448), done.
Starting >>> mros2_echoreply_string
Finished <<< mros2_echoreply_string [9.02s]                     

Summary: 1 package finished [9.17s]
[INFO] [1666065642.412745774] [mros2_echoreply_node]: 
Subscribed msg: 'Hello from mros2-posix onto Linux: 13'
[INFO] [1666065642.413000844] [mros2_echoreply_node]: 
Publishing msg: 'Hello from mros2-posix onto Linux: 13'
[INFO] [1666065643.412568074] [mros2_echoreply_node]: 
Subscribed msg: 'Hello from mros2-posix onto Linux: 14'
[INFO] [1666065643.412670807] [mros2_echoreply_node]: 
Publishing msg: 'Hello from mros2-posix onto Linux: 14'
[INFO] [1666065644.412758317] [mros2_echoreply_node]: 
Subscribed msg: 'Hello from mros2-posix onto Linux: 15'
[INFO] [1666065644.412859042] [mros2_echoreply_node]: 
Publishing msg: 'Hello from mros2-posix onto Linux: 15'
...(SNIPPED)...
```

Here is the example of terminal output for mros2-posix.

```
...(SNIPPED)...
publishing msg: 'Hello from mros2-posix onto Linux: 11'
publishing msg: 'Hello from mros2-posix onto Linux: 12'
LOG_DEBUG : 00000012.806 : [MROS2LIB] subscriber matched with remote publisher
LOG_DEBUG : 00000012.806 : [MROS2LIB] publisher matched with remote subscriber
publishing msg: 'Hello from mros2-posix onto Linux: 13'
subscribed msg: 'Hello from mros2-posix onto Linux: 13'
publishing msg: 'Hello from mros2-posix onto Linux: 14'
subscribed msg: 'Hello from mros2-posix onto Linux: 14'
publishing msg: 'Hello from mros2-posix onto Linux: 15'
subscribed msg: 'Hello from mros2-posix onto Linux: 15'
publishing msg: 'Hello from mros2-posix onto Linux: 16'
subscribed msg: 'Hello from mros2-posix onto Linux: 16'
...(SNIPPED)...
```

You can also confirm the topic by `ros2 topic echo /to_stm` or `ros2 topic echo /to_linux`.

## Examples

This repository contains some example applications in [workspace/](workspace/) to communicate with ROS 2 nodes on the host.
You can switch the example by specifying the second argument of `build.bash`.
Of course you can also create a new program file and specify it as your own application.


Please also check [mROS-base/mros2-host-examples](https://github.com/mROS-base/mros2-host-examples) repository for more detail about the host examples.

### echoback_string

- Description:
  - The mROS 2 node publishes `string` (`std_msgs::msg::String`) message to `/to_linux` topic.
  - (The node on the host will echoreply this message as it is.)
  - The mROS 2 node subscribes the replied message from `/to_stm` topic.
- Host operation:
  - `$ ros2 run mros2_echoreply_string echoreply_node`

### echoreply_string

- Description:
  - The mROS 2 node on the subscribes `string` (`std_msgs::msg::String`) message from `/to_stm` topic.
  - And then publishes this `string` message as it is to `/to_linux` as the reply.
- Host operation:
  - at first terminal: `$ ros2 run mros2_echoback_string sub_node`
  - and then, at second terminal: `$ ros2 run mros2_echoback_string pub_node`
  - or, at one terminal:
    - `$ ros2 launch mros2_echoback_string pubsub.launch.py`

## Submodules and Licenses

The source code of this repository itself is published under Apache License 2.0.
Please note that this repository contains the following stacks as the submodules, and also check their Licenses.

* [mros2](https://github.com/mROS-base/mros2): the pub/sub APIs compatible with ROS 2 Rclcpp
  * [embeddedRTPS](https://github.com/mROS-base/embeddedRTPS): RTPS communication layer (including lwIP and Micro-CDR)
* [cmsis-posix](https://github.com/mROS-base/cmsis-posix): Interface layer between CMSIS OS API and POSIX compiliant kernel (e.g., Linux)
* [lwip-posix](https://github.com/mROS-base/lwip-posix): LwIP UDP/IP implementation based on POSIX compliant kernel (e.g., Linux)
