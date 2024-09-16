#ifndef STRATEGY_MOTIVATION_INTERFACE_H
#define STRATEGY_MOTIVATION_INTERFACE_H

#include <ros/ros.h>

class HDStrategyMotivationInterface {
public:
    HDStrategyMotivationInterface(ros::NodeHandle& nodeHandle);

    void publishMessage(const std::string& data);
    void setCallback(std::function<void(const homodeus_msgs::HDStrategyToMotivation&)> callback);

private:
    ros::Publisher strategy_pub_;
    ros::Subscriber strategy_sub_;
    std::function<void(const homodeus_msgs::HDStrategyToMotivation&)> callback_;

    void subscriberCallback(const homodeus_msgs::HDStrategyToMotivation& msg);
};

#endif 