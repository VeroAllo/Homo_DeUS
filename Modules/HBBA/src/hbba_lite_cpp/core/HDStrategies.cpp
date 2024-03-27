#include <ros/ros.h>
#include <ros/publisher.h>
#include "HDStrategies.h"
#include <hbba_lite/filters/FilterState.h>
#include <hbba_lite/core/Strategy.h>
#include <std_msgs/String.h>
#include <memory>
#include <vector>

class BaseStrategy;

GotoStrategy::GotoStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}


void GotoStrategy::SubscriberCallBack(const std_msgs::String& msg)
{
    if(msg.data == m_desireID)
    {
        onDisabling();
    }
}

void GotoStrategy::Publish(const GotoDesire& desire)
{
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.id();
    onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList){
        pub.publish(msg);
    }
}

TalkStrategy::TalkStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void TalkStrategy::SubscriberCallBack(const std_msgs::String& msg)
{
    if(msg.data == m_desireID)
    {
        onDisabling();
    }
}

void TalkStrategy::Publish(const SpeakDesire& desire)
{    
    std_msgs::String msg;
    msg.data = desire.id();
    onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList){
        pub.publish(msg);
    }
}

DiscussStrategy::DiscussStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}


void DiscussStrategy::SubscriberCallBack(const std_msgs::String& msg)
{
    if(msg.data == m_desireID)
    {
        onDisabling();
    }
}

void DiscussStrategy::Publish(const DiscussDesire& desire)
{
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.id();
    onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList){
        pub.publish(msg);
    }
}

TakeStrategy::TakeStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void TakeStrategy::SubscriberCallBack(const std_msgs::String& msg)
{
    if(msg.data == m_desireID)
    {
        onDisabling();
    }
}

void TakeStrategy::Publish(const TakeDesire& desire)
{
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.id();
    onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList){
        pub.publish(msg);
    }
}

DropStrategy::DropStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void DropStrategy::SubscriberCallBack(const std_msgs::String& msg)
{
    if(msg.data == m_desireID)
    {
        onDisabling();
    }
}

void DropStrategy::Publish(const DropDesire& desire)
{
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.id();
    onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList){
        pub.publish(msg);
    }
}

ExploreStrategy::ExploreStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void ExploreStrategy::SubscriberCallBack(const std_msgs::String& msg)
{
    if(msg.data == m_desireID)
    {
        onDisabling();
    }
}

void ExploreStrategy::Publish(const ExploreDesire& desire)
{
    
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.id();
    onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList){
        pub.publish(msg);
    }
}

std::unique_ptr<BaseStrategy> createGotoStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1)
{
    return std::make_unique<GotoStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{}, std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"goto/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createTalkStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1)
{
    return std::make_unique<TalkStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{},  std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"talk/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createDiscussStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1)
{
    return std::make_unique<DiscussStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{},std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"discuss/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createTakeStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1)
{
    return std::make_unique<TakeStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{},std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"take/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createDropStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1)
{
    return std::make_unique<DropStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{},std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"drop/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createExploreStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1)
{
    return std::make_unique<ExploreStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{}, std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"explore/FilterState", FilterConfiguration::onOff()}});
}