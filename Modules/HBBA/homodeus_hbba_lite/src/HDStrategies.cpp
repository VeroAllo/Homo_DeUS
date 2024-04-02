#include <ros/ros.h>
#include <ros/publisher.h>
#include <homodeus_hbba_lite/HDStrategies.h>
#include <hbba_lite/filters/FilterState.h>
#include <hbba_lite/core/Strategy.h>
#include <std_msgs/String.h>
#include <memory>
#include <vector>

#define PROJECT "/Homodeus"
#define BEHAVIOUR PROJECT "/Behaviour"
#define PERCEPTION PROJECT "/Perception"


GotoStrategy::GotoStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void GotoStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    ROS_INFO("DO WE GO HERE ");
    m_DesireSet->removeDesire(m_desireID);
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO("DO WE GO HERE");
        onDisabling();
    }
}
void GotoStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        //Do stuff
    }
}

void GotoStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    //Do stuff
}

void GotoStrategy::onEnabling(const GotoDesire& desire)
{
    ROS_INFO("DO WE GO PUBLISH");
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.id();
    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(msg);
    }
}

TalkStrategy::TalkStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void TalkStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        onDisabling();
    }
}
void TalkStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        //Do stuff
    }
}

void TalkStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    //Do stuff
}

void TalkStrategy::onEnabling(const TalkDesire& desire)
{    
    std_msgs::String msg;
    msg.data = desire.id();
    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList){
        pub.publish(msg);
    }
}

DiscussStrategy::DiscussStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void DiscussStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        onDisabling();
    }
}
void DiscussStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        //Do stuff
    }
}

void DiscussStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    //Do stuff
}

void DiscussStrategy::onEnabling(const DiscussDesire& desire)
{
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.id();
    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList){
        pub.publish(msg);
    }
}

TakeStrategy::TakeStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void TakeStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        onDisabling();
    }
}
void TakeStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        //Do stuff
    }
}

void TakeStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    //Do stuff
}

void TakeStrategy::onEnabling(const TakeDesire& desire)
{
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.id();
    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList){
        pub.publish(msg);
    }
}

DropStrategy::DropStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void DropStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        onDisabling();
    }
}
void DropStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        //Do stuff
    }
}

void DropStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    //Do stuff
}

void DropStrategy::onEnabling(const DropDesire& desire)
{
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.id();
    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(msg);
    }
}

ExploreStrategy::ExploreStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void ExploreStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        onDisabling();
    }
}
void ExploreStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        //Do stuff
    }
}

void ExploreStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    //Do stuff
}

void ExploreStrategy::onEnabling(const ExploreDesire& desire)
{
    
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.id();
    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList){
        pub.publish(msg);
    }
}

std::unique_ptr<BaseStrategy> createGoToStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<GotoStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Goto/Request", true}, {BEHAVIOUR "/Goto/Cancel", false}}, std::map<std::string, bool>{{BEHAVIOUR "/Goto/Response", false}, {BEHAVIOUR "/Goto/Status", false}}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"goto/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createTalkStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<TalkStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Talk/Request", false}, {BEHAVIOUR "/Talk/Response", false}, {BEHAVIOUR "/Talk/Status", false}},  std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"talk/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createDiscussStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<DiscussStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Discuss/Request", false}, {BEHAVIOUR "/Discuss/Cancel", false}, {BEHAVIOUR "/Discuss/Response", false}, {BEHAVIOUR "/Discuss/Status", false}},std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"discuss/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createTakeStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<TakeStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Take/Request", false}, {BEHAVIOUR "/Take/Cancel", false}, {BEHAVIOUR "/Take/Response", false}, {BEHAVIOUR "/Take/Status", false}},std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"take/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createDropStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<DropStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Drop/Request", false}, {BEHAVIOUR "/Drop/Cancel", false}, {BEHAVIOUR "/Drop/Response", false}, {BEHAVIOUR "/Drop/Status", false}},std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"drop/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createExploreStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<ExploreStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Explore/Request", false}, {BEHAVIOUR "/Explore/Cancel", false}, {BEHAVIOUR "/Explore/Response", false}, {BEHAVIOUR "/Explore/Status", false}}, std::map<std::string, bool>{}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"explore/FilterState", FilterConfiguration::onOff()}});
}