#ifndef STRATEGY_MOTIVATION_INTERFACE_H
#define STRATEGY_MOTIVATION_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/String.h>

class HDStrategyMotivationInterface {
public:
    HDStrategyMotivationInterface(ros::NodeHandle& nodeHandle);

    void publishMessage(const std::string& data);
    void setCallback(std::function<void(std_msgs::String)> callback);

private:
    ros::Publisher strategy_pub_;
    ros::Subscriber strategy_sub_;
    std::function<void(std_msgs::String)> callback_{[](std_msgs::String){}};

    void subscriberCallback(std_msgs::String msg);
};

#endif 