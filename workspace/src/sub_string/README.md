# sub_string
This is a sample application to subscribe `string` message.

## Build and Run

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
