#include "mros2.h"
#include "std_msgs/msg/string.hpp"

#include "cmsis_os.h"
#include "netif.h"
#include "netif_posix_add.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char* argv[])
{
  netif_posix_add(NETIF_IPADDR, NETIF_NETMASK);

  osKernelStart();

  MROS2_INFO("mROS 2 pub application is started");

  mros2::init(0, NULL);
  mros2::Node node = mros2::Node::create_node("mros2_pubnode");
  MROS2_DEBUG("mROS 2 initialization is completed");

  mros2::Publisher pub = node.create_publisher<std_msgs::msg::String>("to_stm", 10);
  MROS2_INFO("ready to publish message");
  auto publish_count = 0;
  while (1) {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello from mROS 2!! " + std::to_string(publish_count++);
    MROS2_INFO("publishing msg: '%s'", msg.data.c_str());
    pub.publish(msg);
    osDelay(1000);
  }

  return 0;
}
