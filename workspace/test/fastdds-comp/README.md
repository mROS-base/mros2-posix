# fastdds-comp
This is a check application for mros2-posix communication layer(embeddedRTPS).

## Build and Run

1. Build mros2-posix on mROS 2 host environments(sender/receiver). Make sure to set fastdds-comp as build.bash option.

```
$ cd mros2-posix
$ bash build.bash clean
$ bash build.bash all fastdds-comp
```

2. Run the mros2-posix with your host `IP address` and `netmask` on mROS 2 host environment(sender).

```
$ ./cmake_build/mros2-posix <IP address> <netmask> send
```

Example:

```
$ ./cmake_build/mros2-posix 192.168.11.49 255.255.255.0 send
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
$ ./cmake_build/mros2-posix <IP address> <netmask>
```

Example:

```
$ ./cmake_build/mros2-posix 192.168.11.50 255.255.255.0
  :
Received Message HelloWorld with index 1
Received Message HelloWorld with index 2
Received Message HelloWorld with index 3
Received Message HelloWorld with index 4
Received Message HelloWorld with index 5
Received Message HelloWorld with index 6
Received Message HelloWorld with index 7
```
