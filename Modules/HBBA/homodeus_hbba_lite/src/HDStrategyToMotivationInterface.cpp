#include <HBBA/homodeus_hbba_lite/include/homodeus_hbba_lite/HDStrategyToMotivationInterface.h>

StrategyMotivationInterface::StrategyMotivationInterface(ros::NodeHandle& nodeHandle) {
    strategy_pub_ = nodeHandle.advertise<homodeus_msgs::StrategyToMotivation>("strategy_to_motivation", 10);
    strategy_sub_ = nodeHandle.subscribe("strategy_to_motivation", 10, &StrategyMotivationInterface::subscriberCallback, this);
}

void StrategyMotivationInterface::publishMessage(const std::string& data) {
    homodeus_msgs::StrategyToMotivation msg;
    msg.data = data;
    strategy_pub_.publish(msg);
}

void StrategyMotivationInterface::setCallback(std::function<void(const homodeus_msgs::StrategyToMotivation&)> callback) {
    callback_ = callback;
}

void StrategyMotivationInterface::subscriberCallback(const homodeus_msgs::StrategyToMotivation& msg) {
    if (callback_) {
        callback_(msg);
    }
}