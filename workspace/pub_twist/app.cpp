#include "mros2.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "cmsis_os.h"
#include "netif.h"
#include "netif_posix_add.h"

#include <stdio.h>
#include <string.h>


int main(int argc, char* argv[])
{
  netif_posix_add(NETIF_IPADDR, NETIF_NETMASK);

  osKernelStart();

  printf("mros2-posix start!\r\n");
  printf("app name: pub_twist\r\n");
  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed\r\n");

  mros2::Node node = mros2::Node::create_node("mros2_node");
  mros2::Publisher pub = node.create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  osDelay(100);
  MROS2_INFO("ready to pub/sub message\r\n");

  geometry_msgs::msg::Vector3 linear;
  geometry_msgs::msg::Vector3 angular;
  geometry_msgs::msg::Twist twist;

  auto publish_count = 0;
  while (1)
  {
    linear.x = publish_count/1.0;
    linear.y = publish_count/1.0;
    linear.z = publish_count/1.0;  
    angular.x = publish_count/1.0;
    angular.y = publish_count/1.0;
    angular.z = publish_count/1.0;
    twist.linear = linear;
    twist.angular = angular;
    MROS2_INFO("publishing Twist msg!!");
    MROS2_INFO("{ linear: {x: %f, y: %f, z: %f }, angular: {x: %f, y: %f, z: %f } }", linear.x, linear.y, linear.z, angular.x, angular.y, angular.z);
    pub.publish(twist);
    publish_count++;
    osDelay(1000);
  }

  mros2::spin();
  return 0;
}
