#include "cmsis_os.h"

#include "mros2.h"
#include "mros2_user_config.h"
#include "std_msgs/msg/string.hpp"

#include "netif_posix_add.h"

#include <array>
#include <chrono>
#include <sstream>

mros2::Publisher pub;

static void userCallback(std_msgs::msg::String *msg) { pub.publish(*msg); }

int echo_main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("usage: %s echo <netmask>\n", argv[0]);
        return 1;
    }
    std::stringstream ipaddr_ss;
    ipaddr_ss << +rtps::Config::IP_ADDRESS[0] << "."
              << +rtps::Config::IP_ADDRESS[1] << "."
              << +rtps::Config::IP_ADDRESS[2] << "."
              << +rtps::Config::IP_ADDRESS[3];
    std::string ipaddr = ipaddr_ss.str();
    MROS2_DEBUG("%s\n", ipaddr.c_str());
    netif_posix_add(ipaddr.c_str(), argv[2]);
    osKernelStart();

    MROS2_INFO("mROS 2 echo application is started");

    mros2::init(argc, argv);
    mros2::Node node = mros2::Node::create_node("mros2_posix_eval_echo");
    MROS2_DEBUG("mROS 2 initialization is completed");
    pub = node.create_publisher<std_msgs::msg::String>("to_linux", 10);
    mros2::Subscriber sub =
        node.create_subscription("to_stm", 10, userCallback);
    MROS2_INFO("ready to subscribe message");
    mros2::spin();
    return 0;
}
