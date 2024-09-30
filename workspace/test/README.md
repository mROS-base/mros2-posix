# Test applications for mros2-posix communication layers

These apps have been developed for testing communication of UDP and RTPS packet, and they were helpful in the eraly stage of development.
However, they are no longer buildable due to major changes in directory structure, etc.
We plan to maintain them in the near future, as they remain useful (see #1).

* lwIP layer: [udp_multicast](https://github.com/mROS-base/mros2-posix/tree/main/workspace/test/udp_multicast)
(embeddedRTPS and lwip-posix)
* embeddedRTPS layer: [fastdds-comp](https://github.com/mROS-base/mros2-posix/tree/main/workspace/test/fastdds-comp)



## mros2 Directory Overview

The `mros2` directory contains several test programs and configuration files, which are actively used in the CI testing process. Below is the structure and overview of the this directory:

```
mros2/
├── docker-compose.yml
├── test_echoback_string
├── test_echoback_string_responder
├── test_echoback_twist
└── test_echoback_twist_responder
```

- **docker-compose.yml**: This is an experimental file that was prepared for setting up the environment, but it is currently not used in the CI process. See `.github/workflows/humble_docker_test.yaml`
- **test_echoback_string**: A test program that sends a string message and expects an echo response, validating communication for string messages in a ROS 2 environment. It is designed to communicate with the `mros2-host-example`'s `mros2_echoreply_string` (a test for Native ROS) and `test_echoback_string_responder` (a test for mROS). The responder listens for incoming messages and sends back the received message.
- **test_echoback_string_responder**: This program subscribes to incoming string messages and echoes them back, facilitating the echo test for string messages.
- **test_echoback_twist**: A test program that sends a `geometry_msgs::msg::Twist` message and expects an echo response, validating communication for Twist messages in a ROS 2 environment. It communicates with the `mros2-host-example`'s `mros2_echoreply_twist` (a test for Native ROS) and `test_echoback_twist_responder` (a test for mROS).
- **test_echoback_twist_responder**: This program subscribes to incoming Twist messages and echoes them back, facilitating the echo test for Twist messages.
