#include <ros/ros.h>
#include <ros/publisher.h>
#include <homodeus_hbba_lite/HDStrategies.h>
#include <homodeus_hbba_lite/HDStrategy.h>
#include <hbba_lite/filters/FilterState.h>
#include <hbba_lite/core/Strategy.h>
#include <../../hbba_state/src/State/GoodbyeState.h>
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
        poseToReturn.position.x = 8.25f;
        poseToReturn.position.y = 6.50f;
        poseToReturn.orientation.z = 2.3562f;
        //  "PosX" :  8.25,
        // "PosY" :  6.50,
        // "PosZ" : 0.00,
        // "ObjOri" : 2.3562
    }
    else if (name == "Table1")
    {        
        poseToReturn.position.x = 8.50f;
        poseToReturn.orientation.z = 3.00f;
        poseToReturn.position.y = 3.09375f;
        // For now, let the default values
    }
    else if (name == "Kitchen")
    {
        
        poseToReturn.position.x = 9.671875f;
        poseToReturn.position.y = 5.78125f;
        poseToReturn.orientation.z = 1.5272f;
        // For now, let the default values
    }
        else if (name == "Home")
    {
        poseToReturn.position.x = 9.0f;
        poseToReturn.position.y = 4.8f;
        poseToReturn.orientation.z = 2.2f;
    }
    return poseToReturn;
}

GotoStrategy::GotoStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName), strategy_motivation_interface_(nodeHandle){}

void GotoStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    if(response.id.desire_id == m_desireID)
    {    
        ROS_INFO_STREAM("GotoDesire Finished - DesireID : " << response.id.desire_id << " - Result : Table 1" );
        m_DesireSet->removeDesire(response.id.desire_id);
        onDisabling();
        return;
    }
}

void GotoStrategy::SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) 
{
   
}

void GotoStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    if(status.id.desire_id == m_desireID)
    {
        // TODO : Implement behaviour in V2
        homodeus_msgs::HDPose hdPose{};
        hdPose.id.desire_id = m_desireID;
        ROS_INFO_STREAM("Destination du desire status: " << status.message.data);
        hdPose.pose = mapStringToPose(status.message.data);
        hdPose.name.data = status.message.data;
        for(ros::Publisher pub : m_PublisherList)
        {
            pub.publish(hdPose);
        }
    }
}

void GotoStrategy::onEnabling(const GotoDesire& desire)
{
    ROS_INFO("GotoDesire started for DesireID : %ld", desire.id());
    ROS_INFO_STREAM("Destination du desire : " << desire.m_DestinationInText);
    m_desireID = desire.id();
    homodeus_msgs::HDPose hdPose{};
    hdPose.id.desire_id = m_desireID;

    hdPose.pose = mapStringToPose(desire.m_DestinationInText);
    hdPose.name.data = desire.m_DestinationInText;
    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(hdPose);
    }
}

TalkStrategy::TalkStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName), strategy_motivation_interface_(nodeHandle){}

void TalkStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{
    ROS_INFO("We got the response, inside was : desire id = %d, other thing = %d", response.id.desire_id, response.result);
    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO_STREAM("TalkDesire Finished - DesireID : " << m_desireID);
        m_DesireSet->removeDesire(m_desireID);
        onDisabling();
        if ((response.message.data.find("Greeting") != std::string::npos) || (response.message.data.find("Bonjour") != std::string::npos))
            strategy_motivation_interface_.publishMessage("Table 1");
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
    textToTalk.message.data = desire.getMessage();

    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(textToTalk);
    }
}

DiscussStrategy::DiscussStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) : HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName), strategy_motivation_interface_(nodeHandle) {}

void DiscussStrategy::SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) 
{

    if(response.id.desire_id == m_desireID)
    {
        ROS_INFO_STREAM("DiscussDesire Finished - DesireID : " << m_desireID);
        m_DesireSet->removeDesire(m_desireID);
        onDisabling();
        strategy_motivation_interface_.publishMessage(response.message.data);
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
    homodeus_msgs::HDDiscussionStarted discussionStarted;
    discussionStarted.id.desire_id = m_desireID;
    discussionStarted.fistMessage.data = desire.getMessage();
    ROS_INFO_STREAM("discussionStarted " << discussionStarted.fistMessage.data);

    for(ros::Publisher pub : m_PublisherList)
    {
        pub.publish(discussionStarted);
    }
}

TakeStrategy::TakeStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet, std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName, StateManager* stateManager) : HDStrategy(filterPool, nodeHandle, publisherTopicList, subscriberTopicList, desireSet, filterConfigurationByName), m_NodeHandle(nodeHandle) 
{
    m_StateManager = stateManager;
}

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
    
}

void TakeStrategy::SubscriberVisionCallback(const homodeus_msgs::ObjectsDetection& objects)
{
    m_ObjectsToDetect = objects;
}

void TakeStrategy::SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) 
{
    if(status.id.desire_id == m_desireID)
    {
        ROS_INFO_STREAM("TakeDesire Status  : " << status.message.data);
        // TODO : Implement behaviour in V2
        std::string fail = "Fail";
        static_cast<GoodbyeState*>(m_StateManager->m_listsStates[2][std::type_index(typeid(GoodbyeState))].get())->generateText(fail);
        m_DesireSet->removeDesire(m_desireID);
        onDisabling();
        return;
    }
}

void TakeStrategy::onEnabling(const TakeDesire& desire)
{
    ROS_INFO("TakeDesire started for DesireID : %ld", desire.id());
    m_desireID = desire.id();

    homodeus_msgs::ObjectDetection boundingBox;
    for (size_t i = 0; i < 3; i++)
    {
        while (m_ObjectsToDetect.objects.size() == 0){ }
        boundingBox = GetClosestTagMatchingCommande(desire.GetCommande());
        if (boundingBox.header.frame_id != "NOT FOUND") break;    
    }
    

    if (boundingBox.header.frame_id == "NOT FOUND")
    {
        ROS_INFO_STREAM(":(");
    }
    else
    {
        boundingBox.id.desire_id = m_desireID;
        // ROS_INFO_STREAM("m_PublisherList : " << m_PublisherList);
        for(ros::Publisher pub : m_PublisherList)
        {
            ROS_INFO_STREAM("Object Detection : " << boundingBox);

            pub.publish(boundingBox);
        }
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

std::unique_ptr<BaseStrategy> createTakeStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, StateManager* stateManager, uint16_t utility)
{
    return std::make_unique<TakeStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Take/Request", false}, {BEHAVIOUR "/Take/Cancel", false}},std::map<std::string, bool>{{BEHAVIOUR "/Take/Response", false}, {BEHAVIOUR "/Take/Status", false}, {PERCEPTION "/Detect", true}}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"Take/FilterState", FilterConfiguration::onOff()}}, stateManager);
}

std::unique_ptr<BaseStrategy> createDropStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<DropStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Drop/Request", false}, {BEHAVIOUR "/Drop/Cancel", false}},std::map<std::string, bool>{{BEHAVIOUR "/Drop/Response", false}, {BEHAVIOUR "/Drop/Status", false}}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"drop/FilterState", FilterConfiguration::onOff()}});
}

std::unique_ptr<BaseStrategy> createExploreStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility)
{
    return std::make_unique<ExploreStrategy>(filterPool, nodeHandle, std::map<std::string, bool>{{BEHAVIOUR "/Explore/Request", false}, {BEHAVIOUR "/Explore/Cancel", false} }, std::map<std::string, bool>{{BEHAVIOUR "/Explore/Response", false}, {BEHAVIOUR "/Explore/Status", false}}, desireSet, std::unordered_map<std::string, FilterConfiguration>{{"explore/FilterState", FilterConfiguration::onOff()}});
}