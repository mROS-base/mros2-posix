#include "cmsis_os.h"

#include "mros2.h"
#include "mros2_user_config.h"
#include "rtps/config.h"
#include "std_msgs/msg/string.hpp"

#include "netif_posix_add.h"

#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>

#define NUM_EVAL 1000

std::array<std::string, NUM_EVAL> sublogs;
std::array<std::chrono::system_clock::time_point, NUM_EVAL> timelogs;

size_t count = 0;
int len_str;
std::string platform;

static void userCallback(std_msgs::msg::String *msg) {
    timelogs[count] = std::chrono::system_clock::now();
    sublogs[count] = msg->data;
    ++count;

    if (count >= NUM_EVAL) {
        std::ofstream writing_file;
        std::string filename = "string" + std::to_string(len_str) + "_sub.csv";
        std::string filepath = "../results/" + platform + "/" + filename;
        writing_file.open(filepath);
        for (int i = 0; i < NUM_EVAL; i++) {
            writing_file
                << sublogs[i] << ","
                << std::to_string(
                       std::chrono::duration_cast<std::chrono::nanoseconds>(
                           timelogs[i].time_since_epoch())
                           .count())
                << std::endl;
        }
        writing_file.close();
        std::exit(0);
    }
}

int sub_main(int argc, char *argv[]) {
    if (argc != 5) {
        printf("Usage: %s sub <platform> <len_str> <netmask>\n", argv[0]);
        return 1;
    }
    std::stringstream ipaddr_ss;
    ipaddr_ss << +rtps::Config::IP_ADDRESS[0] << "."
              << +rtps::Config::IP_ADDRESS[1] << "."
              << +rtps::Config::IP_ADDRESS[2] << "."
              << +rtps::Config::IP_ADDRESS[3];
    std::string ipaddr = ipaddr_ss.str();

    platform = argv[2];
    len_str = std::atoi(argv[3]);
    std::string filename = "string" + std::to_string(len_str) + "_sub.csv";
    std::string filepath = "../results/" + platform + "/" + filename;
    printf("Result output -> %s\n", filepath.c_str());
    netif_posix_add(ipaddr.c_str(), argv[4]);

    osKernelStart();

    mros2::init(argc, argv);
    mros2::Node node = mros2::Node::create_node("mros2_posix_eval_sub");
    mros2::Subscriber sub =
        node.create_subscription("to_linux", 10, userCallback);
    mros2::spin();
    return 0;
}
