#include "cmsis_os.h"

#include "mros2.h"
#include "mros2_user_config.h"
#include "rtps/config.h"
#include "std_msgs/msg/string.hpp"

#include "netif_posix_add.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

#define NUM_EVAL 1000 + 1

class Random_Msg {
  public:
    static std::string get_random(const int len) {
        static const char alphanum[] = "0123456789"
                                       "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                       "abcdefghijklmnopqrstuvwxyz";
        std::string random_string;
        for (int i = 0; i < len; ++i) {
            random_string += alphanum[std::rand() % (sizeof(alphanum) - 1)];
        }
        return random_string;
    }
};

int pub_main(int argc, char *argv[]) {
    if (argc != 5) {
        printf("usage: %s pub <platform> <len_str> <netmask>\n", argv[0]);
        return 1;
    }
    std::stringstream ipaddr_ss;
    ipaddr_ss << +rtps::Config::IP_ADDRESS[0] << "."
              << +rtps::Config::IP_ADDRESS[1] << "."
              << +rtps::Config::IP_ADDRESS[2] << "."
              << +rtps::Config::IP_ADDRESS[3];
    std::string ipaddr = ipaddr_ss.str();

    std::string platform = argv[2];
    int len_str = std::atoi(argv[3]);
    std::string filename = "string" + std::to_string(len_str) + "_pub.csv";
    std::string filepath = "../results/" + platform + "/" + filename;
    printf("Result output -> %s\n", filepath.c_str());
    std::srand(42);
    std::array<std::string, NUM_EVAL> publogs;
    std::array<std::chrono::system_clock::time_point, NUM_EVAL> timelogs;

    netif_posix_add(ipaddr.c_str(), argv[4]);
    osKernelStart();

    mros2::init(argc, argv);
    mros2::Node node = mros2::Node::create_node("mros2_posix_eval_pub");

    mros2::Publisher pub =
        node.create_publisher<std_msgs::msg::String>("to_stm", 10);
    osDelay(10000);
    MROS2_INFO("Evaluation start");
    for (unsigned int i = 0; i < NUM_EVAL; i++) {
        std_msgs::msg::String msg;
        msg.data = Random_Msg::get_random(len_str);
        publogs[i] = msg.data;
        timelogs[i] = std::chrono::system_clock::now();
        pub.publish(msg);
        osDelay(100);
    }
    std::ofstream writing_file;
    writing_file.open(filepath);
    for (int i = 0; i < NUM_EVAL; i++) {
        writing_file
            << publogs[i] << ","
            << std::to_string(
                   std::chrono::duration_cast<std::chrono::nanoseconds>(
                       timelogs[i].time_since_epoch())
                       .count())
            << std::endl;
    }
    writing_file.close();
    MROS2_INFO("Evaluation finished");
    return 0;
}
