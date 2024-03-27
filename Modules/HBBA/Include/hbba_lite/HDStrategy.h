#pragma once

#include <ros/ros.h>
#include <hbba_lite/core/Strategy.h>
#include <hbba_lite/core/DesireSet.h>
#include "HDDesires.h"
#include <ros/node_handle.h>

#include <iostream>
#include <vector>
#include <map>

template<typename T>
class HDStrategy : public Strategy<T>
{
public:
    HDStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string, bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : Strategy<T>(10, {}, filterConfigurationByName, move(filterPool))
    {
        for (const auto& [publisherTopic, latch] : publisherTopicList)
        {
            m_PublisherList.push_back(nodeHandle.advertise<std_msgs::String>(publisherTopic, 10, latch));
        }
        
        for (const auto& [subscriberTopic, latch] : subscriberTopicList)
        {
            m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopic, 10, &HDStrategy<T>::SubscriberCallBack, this));
        }
        m_DesireSet = desireSet;
    }
    std::string m_desireID;
    ~HDStrategy() = default;
    
protected:
    virtual void SubscriberCallBack(const std_msgs::String& msg) = 0;
    std::vector<ros::Publisher>  m_PublisherList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
    std::shared_ptr<DesireSet> m_DesireSet = nullptr;
    void onDisabling() { BaseStrategy::disable(); }
};