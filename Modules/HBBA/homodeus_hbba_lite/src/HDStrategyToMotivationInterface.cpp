#include <HBBA/homodeus_hbba_lite/include/homodeus_hbba_lite/HDStrategyToMotivationInterface.h>

StrategyMotivationInterface::StrategyMotivationInterface(ros::NodeHandle& nodeHandle) {
    strategy_pub_ = nodeHandle.advertise<std_msgs::String&>("strategy_to_motivation", 10);
    strategy_sub_ = nodeHandle.subscribe("strategy_to_motivation", 10, &StrategyMotivationInterface::subscriberCallback, this);
}

void StrategyMotivationInterface::publishMessage(const std::string& data) {
    std_msgs::String& msg;
    msg.data = data;
    strategy_pub_.publish(msg);
}

void StrategyMotivationInterface::setCallback(std::function<void(const std_msgs::String&)> callback) {
    callback_ = callback;
}

void StrategyMotivationInterface::subscriberCallback(const std_msgs::String& msg) {
    if (callback_) {
        callback_(msg);
    }
}