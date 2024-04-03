#include <ros/ros.h>
#include <ros/publisher.h>
#include <homodeus_hbba_lite/HDStrategies.h>
#include <homodeus_hbba_lite/HDStrategy.h>
#include <hbba_lite/filters/FilterState.h>
#include <hbba_lite/core/Strategy.h>
#include <std_msgs/String.h>
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
        poseToReturn.position.x = 2.53;
        poseToReturn.position.y = 0.891;
        poseToReturn.orientation.z = 0.f;
    }
    else if (name == "Table")
    {
        //0 all
    }
    else if (name == "Kitchen")
    {
        //0 all
    }
    return poseToReturn;
}


GotoStrategy::GotoStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void GotoStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO("Goto Finished");
        m_DesireSet->removeDesire(m_desireID);
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
    ROS_INFO("Goto Started %ld", desire.id());
    m_desireID = desire.id();
    homodeus_msgs::HDPose hdPose{};
    hdPose.id.desire_id = m_desireID;
    ROS_INFO_STREAM("Bonjour voici mon but: " << desire.m_DestinationInText);
    hdPose.pose = mapStringToPose(desire.m_DestinationInText);
    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(hdPose);
    }
}

TalkStrategy::TalkStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void TalkStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    ROS_INFO("We got the response, inside was : desire id = %d, other thing = %d", response.id.desire_id, response.result);
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO("Talk Finished");
        m_DesireSet->removeDesire(m_desireID);
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
    ROS_INFO("Talk Started %ld", desire.id());
    m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = "message";
    homodeus_msgs::HDTextToTalk ttt;
    ttt.id.desire_id = m_desireID;
    ttt.message = msg;

    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(ttt);
    }
}

DiscussStrategy::DiscussStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void DiscussStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO("Discuss Finished");
        m_DesireSet->removeDesire(m_desireID);
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
    ROS_INFO("Discuss Started %ld", desire.id());
    m_desireID = desire.id();
    homodeus_msgs::HDDiscussionStarted discussionStarted{};
    discussionStarted.id.desire_id = m_desireID;
    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(discussionStarted);
    }
}

TakeStrategy::TakeStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void TakeStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO("Take finished");
        m_DesireSet->removeDesire(m_desireID);
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
    ROS_INFO("take started %ld", desire.id());
    m_desireID = desire.id();
    homodeus_msgs::HDBoundingBox hbBB{};
    hbBB.id.desire_id = m_desireID;
    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(hbBB);
    }
}

DropStrategy::DropStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void DropStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO("drop finished");
        m_DesireSet->removeDesire(m_desireID);
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
    ROS_INFO("Drop started %ld", desire.id());
    m_desireID = desire.id();
    homodeus_msgs::HDBoundingBox hbBB{};
    hbBB.id.desire_id = m_desireID;
    //onEnabling(desire);
    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(hbBB);
    }
}

ExploreStrategy::ExploreStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) 
: HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName){}

void ExploreStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO("Explore finished");
        m_DesireSet->removeDesire(m_desireID);
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
    ROS_INFO("explore started %ld", desire.id());
    m_desireID = desire.id();
    homodeus_msgs::DesireID desireID{};
    desireID.desire_id = m_desireID;
    //onEnabling(desire);
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