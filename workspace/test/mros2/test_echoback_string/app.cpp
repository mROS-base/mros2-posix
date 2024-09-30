
/**
 * このプログラムとechoreply_stringを使う
 * 10回くらい送信して返ってくるかチェックする
 */

#include "mros2.h"
#include "std_msgs/msg/string.hpp"

#include "cmsis_os.h"
#include "netif.h"
#include "netif_posix_add.h"

#include <stdio.h>
#include <string.h>
#include <pthread.h>

bool result = true;
auto TestMessagePreFix = "TestMessage";
std::string expect = "";
int sub_counter = 0;
pthread_mutex_t mutex_expect_sting;

void userCallback(std_msgs::msg::String *msg)
{
  pthread_mutex_lock(&mutex_expect_sting);
  sub_counter++;
  printf("*test count : %d\r\n", sub_counter);
  auto actual = msg->data;
  // FIXME 受信データに改行や不要な文字が含まれている模様
  actual.erase(std::remove(actual.begin(), actual.end(), '\n'), actual.end());
  actual.erase(std::remove(actual.begin(), actual.end(), '\r'), actual.end());

  MROS2_DEBUG("Sub result:\r\n expect : '%s',\r\n actural: '%s'", expect.c_str(), actual.c_str());
  MROS2_DEBUG("expect length: %lu, actual length: %lu", expect.size(), actual.size());
  if (strcmp(expect.c_str(), actual.c_str()) == 0)
  {
    MROS2_INFO("****SUCCEED****");
    result = result & true;
  }
  else
  {
    MROS2_INFO("****FAIL different data responsed.***");
    result = result & false;
  }
  pthread_mutex_unlock(&mutex_expect_sting);
  if (sub_counter > 10)
  {
    if (result)
    {
      MROS2_INFO("All tests succeed !!");
      std::exit(0);
    }
    MROS2_INFO("Some tests faild.");
    std::exit(-1);
  }
}


int main(int argc, char *argv[])
{
  netif_posix_add(NETIF_IPADDR, NETIF_NETMASK);

  osKernelStart();

  MROS2_INFO("mros2-posix start!\r\n");
  MROS2_INFO("app name: echoback_string\r\n");
  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed\r\n");

  mros2::Node node = mros2::Node::create_node("mros2_test_node");
  mros2::Publisher pub = node.create_publisher<std_msgs::msg::String>("to_linux", 10);
  auto msg = std_msgs::msg::String();
  msg.data = "initial";
  pub.publish(msg);
  osDelay(1000);
  mros2::Subscriber sub = node.create_subscription<std_msgs::msg::String>("to_stm", 10, userCallback);

  MROS2_INFO("ready to pub/sub message\r\n");
  auto count = 0;
  sub_counter = 0;
  while (true)
  {
    auto msg = std_msgs::msg::String();
    pthread_mutex_lock(&mutex_expect_sting);
    expect = TestMessagePreFix + std::to_string(count++);
    msg.data = expect;
    MROS2_INFO("publishing msg: '%s'", msg.data.c_str());
    pub.publish(msg);
    pthread_mutex_unlock(&mutex_expect_sting);

    osDelay(1000);
  }

  mros2::spin();
  return 0;
}
