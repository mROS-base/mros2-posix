#include "cmsis_os.h"

#include "mros2.h"
#include "mros2_user_config.h"
#include "std_msgs/msg/string.hpp"

#include "netif_posix_add.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char* argv[])
{
	if (argc != 3) {
		printf("Usage: %s <ipaddr> <netmask>\n", argv[0]);
		return 1;
	}
	netif_posix_add(argv[1], argv[2]);

	osKernelStart();

	MROS2_INFO("mROS 2 pub application is started");

	mros2::init(argc, argv);
	mros2::Node node = mros2::Node::create_node("mros2_pubnode");
	MROS2_DEBUG("mROS 2 initialization is completed");

	mros2::Publisher pub = node.create_publisher<std_msgs::msg::String>("to_stm", 10);
	MROS2_INFO("ready to publish message");
	std_msgs::msg::String msg;
	auto publish_count = 0;
	while (1) {
		msg.data = "Hello from mROS 2!! " + std::to_string(publish_count++);
		MROS2_INFO("publishing msg: '%s'", msg.data.c_str());
		pub.publish(msg);
		osDelay(1000);
	}

	return 0;
}
int mros2_get_submsg_count(void)
{
	return 10;
}
