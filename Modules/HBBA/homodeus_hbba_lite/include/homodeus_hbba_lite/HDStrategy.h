#pragma once

#include <ros/ros.h>
#include <hbba_lite/core/Strategy.h>
#include <hbba_lite/core/DesireSet.h>
#include "HDDesires.h"
#include <ros/node_handle.h>

//Les messages homodeus
#include <homodeus_msgs/DesireID.h>
#include <homodeus_msgs/HDResponse.h>
#include <homodeus_msgs/HDStatus.h>
#include <homodeus_msgs/HDPose.h>
#include <homodeus_msgs/HDDiscussionStarted.h>
#include <homodeus_msgs/HDTextToTalk.h>
#include <homodeus_msgs/HDBoundingBox.h>

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
            m_PublisherList.push_back(generatePub(nodeHandle, publisherTopic, latch));
        }
        
        for (const std::pair<const std::string, bool>& pair : subscriberTopicList)
        {
            const std::string& subscriberTopic = pair.first;
            m_SubscriberList.push_back(generateSub(nodeHandle, subscriberTopic));
        }
    }
    uint16_t m_desireID;
    ~HDStrategy() = default;
    
protected:
    virtual void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) = 0;
    virtual void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) = 0;
    virtual void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) = 0;
    std::vector<ros::Publisher>  m_PublisherList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
    std::shared_ptr<DesireSet> m_DesireSet = nullptr;
    void onDisabling() { BaseStrategy::disable(); }

private:
    ros::Publisher generatePub(ros::NodeHandle& nh, std::string str, bool latch)
    {
        if (str.find("Request"))
        {
            if (str.find("Goto"))
            {
                return nh.advertise<homodeus_msgs::HDPose>(str, 10, latch);
            }
            else if (str.find("Talk"))
            {
                return nh.advertise<homodeus_msgs::HDTextToTalk>(str, 10, latch);
            }
            else if (str.find("Discuss"))
            {
                return nh.advertise<homodeus_msgs::HDDiscussionStarted>(str, 10, latch);
            }
            else if (str.find("Take"))
            {
                return nh.advertise<homodeus_msgs::HDBoundingBox>(str, 10, latch);
            }
            else if (str.find("Drop"))
            {
                return nh.advertise<homodeus_msgs::HDBoundingBox>(str, 10, latch);
            }
            else if (str.find("Explore"))
            {
                return nh.advertise<homodeus_msgs::DesireID>(str, 10, latch);
            }
        }
        return nh.advertise<std_msgs::String>("Failed", 10, true);
    }

ros::Subscriber generateSub(ros::NodeHandle& nh, std::string str)
    {
        if (str.find("Response"))
        {
            return nh.subscribe(str, 10, &HDStrategy<T>::SubscriberResponseCallBack, this);
        }
        else if (str.find("Status"))
        {
            return nh.subscribe(str, 10, &HDStrategy<T>::SubscriberStatusCallBack, this);
        }
        else if (str.find("Cancel"))
        {
            return nh.subscribe(str, 10, &HDStrategy<T>::SubscriberCancelCallBack, this);
        }
        return nh.subscribe("Failed", 1, &HDStrategy<T>::SubscriberCancelCallBack, this);
    }

};