#include <ros/ros.h>
#include <ros/publisher.h>
#include <homodeus_hbba_lite/HDStrategies.h>
#include <homodeus_hbba_lite/HDStrategy.h>
#include <hbba_lite/filters/FilterState.h>
#include <hbba_lite/core/Strategy.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Pose.h>
#include <homodeus_msgs/HDPose.h>
#include <homodeus_msgs/HDTextToTalk.h>
#include <homodeus_msgs/HDDiscussionStarted.h>
#include <homodeus_msgs/DesireID.h>
#include <memory>
#include <vector>

#define PROJECT "/Homodeus"
#define BEHAVIOUR PROJECT "/Behaviour"
#define PERCEPTION PROJECT "/Perception"

geometry_msgs::Pose mapStringToPose(std::string name) 
{
    geometry_msgs::Pose poseToReturn{};
    if (name == "Accueil") 
    {
        poseToReturn.position.x = 2.53f;
        poseToReturn.position.y = 0.891f;
    }
    else if (name == "Table")
    {        
        poseToReturn.position.x = 0.21f;
        poseToReturn.orientation.z = -0.79f;

        // For now, let the default values
    }
    else if (name == "Kitchen")
    {
        // For now, let the default values
    }
    return poseToReturn;
}

GotoStrategy::GotoStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName), strategy_motivation_interface_(nodeHandle){}

void GotoStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO_STREAM("GotoDesire Finished - DesireID : " << m_desireID << " - Result : " << response.result);
        m_DesireSet->removeDesire(m_desireID);
        onDisabling();
        strategy_motivation_interface_.publishMessage(response.result);
        return;
    }
    ROS_ERROR_STREAM("The desireIDs do not match - Received : " << response.id.desire_id << ", Expected : " << m_desireID);
}

void GotoStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        // TODO : Implement behaviour in V2
    }
}

void GotoStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    // TODO : Implement behaviour in V2
}

void GotoStrategy::onEnabling(const GotoDesire& desire)
{
    ROS_INFO("GotoDesire started for DesireID : %ld", desire.id());
    ROS_INFO_STREAM("Destination du desire : " << desire.m_DestinationInText);
    m_desireID = desire.id();
    homodeus_msgs::HDPose hdPose{};
    hdPose.id.desire_id = m_desireID;
    hdPose.pose = mapStringToPose(desire.m_DestinationInText);
    
    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(hdPose);
    }
}

TalkStrategy::TalkStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void TalkStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    ROS_INFO("We got the response, inside was : desire id = %d, other thing = %d", response.id.desire_id, response.result);
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO_STREAM("TalkDesire Finished - DesireID : " << m_desireID);
        m_DesireSet->removeDesire(m_desireID);
        onDisabling();
        return;
    }
    ROS_ERROR_STREAM("The desireIDs do not match - Received : " << response.id.desire_id << ", Expected : " << m_desireID);
}

void TalkStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        // TODO : Implement behaviour in V2
    }
}

void TalkStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    // TODO : Implement behaviour in V2
}

void TalkStrategy::onEnabling(const TalkDesire& desire)
{   
    ROS_INFO("TalkDesire started for DesireID : %ld", desire.id());
    m_desireID = desire.id();
    homodeus_msgs::HDTextToTalk textToTalk;
    textToTalk.id.desire_id = m_desireID;
    textToTalk.message = m_TextToTalk;

    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(textToTalk);
    }
}

DiscussStrategy::DiscussStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void DiscussStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO_STREAM("DiscussDesire Finished - DesireID : " << m_desireID);
        m_DesireSet->removeDesire(m_desireID);
        onDisabling();
        return;
    }
    ROS_ERROR_STREAM("The desireIDs do not match - Received : " << response.id.desire_id << ", Expected : " << m_desireID);
}

void DiscussStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        // TODO : Implement behaviour in V2
    }
}

void DiscussStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    // TODO : Implement behaviour in V2
}

void DiscussStrategy::onEnabling(const DiscussDesire& desire)
{
    ROS_INFO("DiscussDesire started for DesireID : %ld", desire.id());
    m_desireID = desire.id();
    homodeus_msgs::HDDiscussionStarted discussionStarted{};
    discussionStarted.id.desire_id = m_desireID;
    discussionStarted.fistMessage = "Welcome to Billy Bob Buger"

    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(discussionStarted);
    }
}

TakeStrategy::TakeStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName) {}

void TakeStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{    
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO_STREAM("TakeDesire Finished - DesireID : " << m_desireID);
        m_DesireSet->removeDesire(m_desireID);
        onDisabling();
        return;
    }
    ROS_ERROR_STREAM("The desireIDs do not match - Received : " << response.id.desire_id << ", Expected : " << m_desireID);
}

void TakeStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        // TODO : Implement behaviour in V2
    }
}

void TakeStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    // TODO : Implement behaviour in V2
}

void TakeStrategy::onEnabling(const TakeDesire& desire)
{
    ROS_INFO("TakeDesire started for DesireID : %ld", desire.id());
    m_desireID = desire.id();
    homodeus_msgs::HDBoundingBox boundingBox{};
    boundingBox.id.desire_id = m_desireID;

    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(boundingBox);
    }
}

DropStrategy::DropStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void DropStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO_STREAM("DropDesire Finished - DesireID : " << m_desireID);
        m_DesireSet->removeDesire(m_desireID);
        onDisabling();
        return;
    }
    ROS_ERROR_STREAM("The desireIDs do not match - Received : " << response.id.desire_id << ", Expected : " << m_desireID);
}

void DropStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        // TODO : Implement behaviour in V2
    }
}

void DropStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    // TODO : Implement behaviour in V2
}

void DropStrategy::onEnabling(const DropDesire& desire)
{
    ROS_INFO("DropDesire started for DesireID : %ld", desire.id());
    m_desireID = desire.id();
    homodeus_msgs::HDBoundingBox boundingBox{};
    boundingBox.id.desire_id = m_desireID;

    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(boundingBox);
    }
}

ExploreStrategy::ExploreStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void ExploreStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO_STREAM("ExploreDesire Finished - DesireID : " << m_desireID);
        m_DesireSet->removeDesire(m_desireID);
        onDisabling();
        return;
    }
    ROS_ERROR_STREAM("The desireIDs do not match - Received : " << response.id.desire_id << ", Expected : " << m_desireID);
}

void ExploreStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
    if(desireID.desire_id == m_desireID)
    {
        // TODO : Implement behaviour in V2
    }
}

void ExploreStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    // TODO : Implement behaviour in V2
}

void ExploreStrategy::onEnabling(const ExploreDesire& desire)
{
    ROS_INFO_STREAM("ExploreDesire started for DesireID : " << desire.id());
    homodeus_msgs::DesireID desireID{};
    m_desireID = desire.id();
    desireID.desire_id = m_desireID;

    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(desireID);
    }
}

std::unique_ptr<BaseStrategy> createGoToStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<GotoStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Goto/Request", false}, {BEHAVIOUR "/Goto/Cancel", false}}, std::map<std::string, bool>{{BEHAVIOUR "/Goto/Response", false}, {BEHAVIOUR "/Goto/Status", false}}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"goto/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createTalkStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<TalkStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Talk/Request", false} },  std::map<std::string, bool>{{BEHAVIOUR "/Talk/Response", false}, {BEHAVIOUR "/Talk/Status", false}}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"talk/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createDiscussStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<DiscussStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Discuss/Request", false}, {BEHAVIOUR "/Discuss/Cancel", false} },std::map<std::string, bool>{{BEHAVIOUR "/Discuss/Response", false}, {BEHAVIOUR "/Discuss/Status", false}}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"discuss/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createTakeStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<TakeStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Take/Request", false}, {BEHAVIOUR "/Take/Cancel", false}},std::map<std::string, bool>{{BEHAVIOUR "/Take/Response", false}, {BEHAVIOUR "/Take/Status", false}}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"take/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createDropStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<DropStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Drop/Request", false}, {BEHAVIOUR "/Drop/Cancel", false}},std::map<std::string, bool>{{BEHAVIOUR "/Drop/Response", false}, {BEHAVIOUR "/Drop/Status", false}}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"drop/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createExploreStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<ExploreStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Explore/Request", false}, {BEHAVIOUR "/Explore/Cancel", false} }, std::map<std::string, bool>{{BEHAVIOUR "/Explore/Response", false}, {BEHAVIOUR "/Explore/Status", false}}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"explore/FilterState", FilterConfiguration::onOff()}});
}