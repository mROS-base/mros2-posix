
/**
 * このプログラムとechoreply_twistを使う
 * 10回くらい送信して返ってくるかチェックする
 */

#include "mros2.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "cmsis_os.h"
#include "netif.h"
#include "netif_posix_add.h"

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <math.h>

bool result = true;
auto TestMessagePreFix = "TestMessage";
geometry_msgs::msg::Twist expect;
int sub_counter = 0;
pthread_mutex_t mutex_expect_twist;

void userCallback(geometry_msgs::msg::Twist *msg)
{
  pthread_mutex_lock(&mutex_expect_twist);
  sub_counter++;
  MROS2_INFO("*test count : %d\r\n", sub_counter);

  auto actual = *msg;
  bool pub_sub_result = true;

  MROS2_INFO("expect { linear: {x: %f, y: %f, z: %f }, angular: {x: %f, y: %f, z: %f } }",
    expect.linear.x, expect.linear.y, expect.linear.z, expect.angular.x, expect.angular.y, expect.angular.z);
  MROS2_INFO("actual { linear: {x: %f, y: %f, z: %f }, angular: {x: %f, y: %f, z: %f } }",
     actual.linear.x, actual.linear.y, actual.linear.z, actual.angular.x, actual.angular.y, actual.angular.z);

  pub_sub_result = pub_sub_result & (abs(actual.linear.x - expect.linear.x) < 0.001);
  pub_sub_result = pub_sub_result & (abs(actual.linear.y - expect.linear.y) < 0.001);
  pub_sub_result = pub_sub_result & (abs(actual.linear.z - expect.linear.z) < 0.001);
  pub_sub_result = pub_sub_result & (abs(actual.angular.x - expect.angular.x) < 0.001);
  pub_sub_result = pub_sub_result & (abs(actual.angular.y - expect.angular.y) < 0.001);
  pub_sub_result = pub_sub_result & (abs(actual.angular.z - expect.angular.z) < 0.001);

  if (pub_sub_result)
  {
    MROS2_INFO("****SUCCEED****'\r\n");
    result = result & true;
  }
  else
  {
    MROS2_INFO("FAIL different data responsed.\r\n");
    result = result & false;
  }
  pthread_mutex_unlock(&mutex_expect_twist);
  MROS2_INFO("\r\n");

  if (sub_counter > 10)
  {
    if (result)
    {
      MROS2_INFO("All tests succeed.\r\n");
      std::exit(0);
    }
    MROS2_INFO("Some tests faild.\r\n");
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
  mros2::Publisher pub = node.create_publisher<geometry_msgs::msg::Twist>("to_linux", 10);
  osDelay(1000);
  mros2::Subscriber sub = node.create_subscription<geometry_msgs::msg::Twist>("to_stm", 10, userCallback);

  MROS2_INFO("ready to pub/sub message\r\n");

  geometry_msgs::msg::Vector3 linear;
  geometry_msgs::msg::Vector3 angular;

  auto count = 0;
  sub_counter = 0;
  auto publish_count = 0;
  while (true)
  {
    pthread_mutex_lock(&mutex_expect_twist);
    linear.x = publish_count/1.0;
    linear.y = publish_count/1.0;
    linear.z = publish_count/1.0;
    angular.x = publish_count/1.0;
    angular.y = publish_count/1.0;
    angular.z = publish_count/1.0;
    expect.linear = linear;
    expect.angular = angular;
    MROS2_INFO("publishing Twist msg!!");
    MROS2_INFO("{ linear: {x: %f, y: %f, z: %f }, angular: {x: %f, y: %f, z: %f } }", linear.x, linear.y, linear.z, angular.x, angular.y, angular.z);
    pub.publish(expect);
    publish_count++;
    pthread_mutex_unlock(&mutex_expect_twist);
    osDelay(1000);
  }

  mros2::spin();
  return 0;
}
