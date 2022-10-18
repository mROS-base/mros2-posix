#include "cmsis_os.h"

#include "mros2.h"
#include "geometry_msgs/msg/pose.hpp"

#include "netif.h"
#include "netif_posix_add.h"

#include <stdio.h>
#include <string.h>


void userCallback(geometry_msgs::msg::Pose *msg)
{
  MROS2_INFO("subscribed Pose msg!!");
  MROS2_INFO("{ position: {x: %f, y: %f, z: %f }, orientation: {x: %f, y: %f, z: %f, w: %f } }", msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

int main(int argc, char* argv[])
{
  netif_posix_add(NETIF_IPADDR, NETIF_NETMASK);

  osKernelStart();

  printf("mros2-posix start!\r\n");
  printf("app name: sub_pose\r\n");
  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed\r\n");

  mros2::Node node = mros2::Node::create_node("sub_pose");
  mros2::Subscriber sub = node.create_subscription<geometry_msgs::msg::Pose>("cmd_vel", 10, userCallback);
  osDelay(100);
  MROS2_INFO("ready to pub/sub message");

  mros2::spin();
  return 0;
}
