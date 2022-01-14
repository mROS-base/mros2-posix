#include "cmsis_os.h"

#include "mros2.h"
#include "mros2_user_config.h"
#include "std_msgs/msg/string.hpp"

#include "netif_posix_add.h"

#include <stdio.h>
#include <string.h>

static void userCallback(std_msgs::msg::String* msg)
{
	MROS2_INFO("subscribed msg: '%s'", msg->data.c_str());
}

int main(int argc, char* argv[])
{
	if (argc != 3) {
		printf("Usage: %s <ipaddr> <netmask>\n", argv[0]);
		return 1;
	}
	netif_posix_add(argv[1], argv[2]);

	osKernelStart();

	MROS2_INFO("mROS 2 sub application is started");

	mros2::init(argc, argv);
	mros2::Node node = mros2::Node::create_node("mros2_subnode");
	MROS2_DEBUG("mROS 2 initialization is completed");
	mros2::Subscriber sub = node.create_subscription("to_stm", 10, userCallback);
	MROS2_INFO("ready to subscribe message");
	mros2::spin();

	return 0;
}
int mros2_get_submsg_count(void)
{
	return 10;
}
