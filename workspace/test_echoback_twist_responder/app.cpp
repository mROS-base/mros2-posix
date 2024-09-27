#include "mros2.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"


#include "cmsis_os.h"
#include "netif.h"
#include "netif_posix_add.h"

#include <stdio.h>
#include <string.h>


mros2::Subscriber sub;
mros2::Publisher pub;

void userCallback(geometry_msgs::msg::Twist *msg)
{
  MROS2_INFO("expect { linear: {x: %f, y: %f, z: %f }, angular: {x: %f, y: %f, z: %f } }",
    msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
  pub.publish(*msg);
}

int main(int argc, char* argv[])
{
  netif_posix_add(NETIF_IPADDR, NETIF_NETMASK);

  osKernelStart();

  printf("mros2-posix start!\r\n");
  printf("app name: echoreply_string\r\n");
  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed\r\n");

  mros2::Node node = mros2::Node::create_node("mros2_node");
  pub = node.create_publisher<geometry_msgs::msg::Twist>("to_stm", 10);
  sub = node.create_subscription<geometry_msgs::msg::Twist>("to_linux", 10, userCallback);
  osDelay(100);
  MROS2_INFO("ready to pub/sub message\r\n");

  mros2::spin();
  return 0;
}
