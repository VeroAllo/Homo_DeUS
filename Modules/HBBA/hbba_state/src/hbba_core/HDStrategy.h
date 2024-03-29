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
    HDStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string, bool> publisherTopicList, std::map<std::string, bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : Strategy<T>(10, {}, filterConfigurationByName, move(filterPool)), m_DesireSet(desireSet)
    {
        for (const std::pair<const std::string, bool>& pair : publisherTopicList)
        {
            const std::string& publisherTopic = pair.first;
            const bool& latch = pair.second;
            m_PublisherList.push_back(nodeHandle.advertise<std_msgs::String>(publisherTopic, 10, latch));
        }
        
        for (const std::pair<const std::string, bool>& pair : subscriberTopicList)
        {
            const std::string& subscriberTopic = pair.first;
            m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopic, 10, &HDStrategy<T>::SubscriberCallBack, this));
        }
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