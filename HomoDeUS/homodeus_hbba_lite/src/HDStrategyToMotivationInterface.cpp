#include <homodeus_hbba_lite/HDStrategyToMotivationInterface.h>
#include <std_msgs/String.h>

HDStrategyMotivationInterface::HDStrategyMotivationInterface(ros::NodeHandle& nodeHandle) 
{
    strategy_pub_ = nodeHandle.advertise<std_msgs::String>("strategy_to_motivation", 10);
    strategy_sub_ = nodeHandle.subscribe("strategy_to_motivation", 10, &HDStrategyMotivationInterface::subscriberCallback, this);
}

void HDStrategyMotivationInterface::publishMessage(const std::string& data) 
{
    std_msgs::String msg{};
    msg.data = data;
    strategy_pub_.publish(msg);
}

void HDStrategyMotivationInterface::setCallback(std::function<void(std_msgs::String)> callback)
{
    callback_ = callback;
}

void HDStrategyMotivationInterface::subscriberCallback(std_msgs::String msg)
{
    if (callback_)
    {
        callback_(msg);
    }
}