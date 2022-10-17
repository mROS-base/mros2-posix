
# udp_multicast

This is a check application for mros2-posix communication layer(lwip-posix udp multicast).

## Build and Run

1. Build mros2-posix on mROS 2 host environments(sender/receiver). Make sure to set udp_multicast as build.bash option.

```
$ cd mros2-posix
$ bash build.bash clean
$ bash build.bash all udp_multicast
```

2. Run the mros2-posix with your host `IP address` and `netmask` on mROS 2 host environment(sender).

```
$ ./cmake_build/mros2-posix <IP address> <netmask> send
```

Example:

```
$ ./cmake_build/mros2-posix 192.168.11.49 255.255.255.0 send
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
$ ./cmake_build/mros2-posix <IP address> <netmask>
```

Example:

```
$ ./cmake_build/mros2-posix 192.168.11.50 255.255.255.0
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
