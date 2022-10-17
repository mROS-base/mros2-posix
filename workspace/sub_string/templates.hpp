
#include "std_msgs/msg/string.hpp"




template mros2::Subscriber mros2::Node::create_subscription(std::string topic_name, int qos, void (*fp)(std_msgs::msg::String*));
template void mros2::Subscriber::callback_handler<std_msgs::msg::String>(void *callee, const rtps::ReaderCacheChange &cacheChange);
