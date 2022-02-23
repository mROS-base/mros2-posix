# Example applications for mros2-posix

This repository contains two types of applications:

* Sample applications for mros2-posix
* Check applications for mros2-posix communication layers(embeddedRTPS and lwip-posix) 

## Sample applications for mros2-posix

### pub_string
This is a sample application to publish `string` message. Please refer to [here](https://github.com/mROS-base/mros2-posix#build-for-mros-2-app) for the build and run procedure.

### sub_string
This is a sample application to subscribe `string` message.

#### Build and Run

1. Build mros2-posix on mROS 2 host environment. Make sure to set sub_string as build.bash option.

```
$ cd mros2-posix
$ bash build.bash sub_string clean
$ bash build.bash sub_string all
```

2. Launch ROS 2 topic pub node on ROS 2 host environment.

```
$ source /opt/ros/foxy/setup.bash
$ ros2 topic pub /to_stm std_msgs/String '{data: hello world}'
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='hello world')

publishing #2: std_msgs.msg.String(data='hello world')

publishing #3: std_msgs.msg.String(data='hello world')

publishing #4: std_msgs.msg.String(data='hello world')
```

3. Run the mros2-posix with your host IP address and netmask on mROS 2 host environment.

```
$ ./cmake-build/mros2-posix <IP address> <netmask>
```

Example:

```
$ ./cmake-build/mros2-posix 192.168.11.49 255.255.255.0
  :
LOG_NOTICE : 00000000.136 : thread_udp_recv:UP: mcp=0x7f7290000e20
LOG_DEBUG : 00000000.137 : [MROS2LIB] successfully created participant
LOG_DEBUG : 00000000.137 : mROS 2 initialization is completed
LOG_DEBUG : 00000000.137 : [MROS2LIB] create_subscription complete
LOG_NOTICE : 00000000.138 : ready to subscribe message
LOG_DEBUG : 00000000.230 : [MROS2LIB] Initilizing Domain complete
LOG_NOTICE : 00000000.484 : subscribed msg: 'hello world'
LOG_NOTICE : 00000001.484 : subscribed msg: 'hello world'
LOG_NOTICE : 00000002.484 : subscribed msg: 'hello world'
LOG_NOTICE : 00000003.484 : subscribed msg: 'hello world'
LOG_NOTICE : 00000004.483 : subscribed msg: 'hello world'
```

## Check applications for mros2-posix communication layers(embeddedRTPS and lwip-posix) 

### fastdds-comp
This is a check application for mros2-posix communication layer(embeddedRTPS).

#### Build and Run

1. Build mros2-posix on mROS 2 host environments(sender/receiver). Make sure to set fastdds-comp as build.bash option.

```
$ cd mros2-posix
$ bash build.bash fastdds-comp clean
$ bash build.bash fastdds-comp all
```

2. Run the mros2-posix with your host `IP address` and `netmask` on mROS 2 host environment(sender).

```
$ ./cmake-build/mros2-posix <IP address> <netmask> send
```

Example:

```
$ ./cmake-build/mros2-posix 192.168.11.49 255.255.255.0 send
  :
Sending HelloWorldPackets: 10
Hello WorldPacket sent: Index 1
Hello WorldPacket sent: Index 2
Hello WorldPacket sent: Index 3
Hello WorldPacket sent: Index 4
Hello WorldPacket sent: Index 5
Hello WorldPacket sent: Index 6
Hello WorldPacket sent: Index 7
```

3. Run the mros2-posix with your host `IP address` and `netmask` on mROS 2 host environment(receiver).

```
$ ./cmake-build/mros2-posix <IP address> <netmask>
```

Example:

```
$ ./cmake-build/mros2-posix 192.168.11.50 255.255.255.0
  :
Received Message HelloWorld with index 1
Received Message HelloWorld with index 2
Received Message HelloWorld with index 3
Received Message HelloWorld with index 4
Received Message HelloWorld with index 5
Received Message HelloWorld with index 6
Received Message HelloWorld with index 7
```

### udp_multicast

This is a check application for mros2-posix communication layer(lwip-posix udp multicast).

#### Build and Run

1. Build mros2-posix on mROS 2 host environments(sender/receiver). Make sure to set udp_multicast as build.bash option.

```
$ cd mros2-posix
$ bash build.bash udp_multicast clean
$ bash build.bash udp_multicast all
```

2. Run the mros2-posix with your host `IP address` and `netmask` on mROS 2 host environment(sender).

```
$ ./cmake-build/mros2-posix <IP address> <netmask> send
```

Example:

```
$ ./cmake-build/mros2-posix 192.168.11.49 255.255.255.0 send
  :
LOG_NOTICE : 00000000.007 : send data:port=7401 err=0
LOG_NOTICE : 00000001.022 : send data:port=7401 err=0
LOG_NOTICE : 00000002.035 : send data:port=7401 err=0
LOG_NOTICE : 00000003.055 : send data:port=7401 err=0
LOG_NOTICE : 00000004.063 : send data:port=7401 err=0
LOG_NOTICE : 00000005.067 : send data:port=7401 err=0
LOG_NOTICE : 00000006.071 : send data:port=7401 err=0
LOG_NOTICE : 00000007.073 : send data:port=7401 err=0
```

3. Run the mros2-posix with your host `IP address` and `netmask` on mROS 2 host environment(receiver).

```
$ ./cmake-build/mros2-posix <IP address> <netmask>
```

Example:

```
$ ./cmake-build/mros2-posix 192.168.11.50 255.255.255.0
  :
LOG_NOTICE : 00000002.649 : recv data:addr=192.168.11.49 port=59676 payload=Hello World!!
LOG_NOTICE : 00000003.663 : recv data:addr=192.168.11.49 port=59676 payload=Hello World!!
LOG_NOTICE : 00000004.676 : recv data:addr=192.168.11.49 port=59676 payload=Hello World!!
LOG_NOTICE : 00000005.697 : recv data:addr=192.168.11.49 port=59676 payload=Hello World!!
LOG_NOTICE : 00000006.705 : recv data:addr=192.168.11.49 port=59676 payload=Hello World!!
LOG_NOTICE : 00000007.709 : recv data:addr=192.168.11.49 port=59676 payload=Hello World!!
LOG_NOTICE : 00000008.712 : recv data:addr=192.168.11.49 port=59676 payload=Hello World!!
LOG_NOTICE : 00000009.714 : recv data:addr=192.168.11.49 port=59676 payload=Hello World!!
```
