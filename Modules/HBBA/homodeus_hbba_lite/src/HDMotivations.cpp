#include <homodeus_hbba_lite/HDMotivations.h>
#include <homodeus_msgs/ObjectDetection.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>

#define TIMEAUQUELTABLEAETEPRISE 1
#define PROJECT "/Homodeus"
#define BEHAVIOUR PROJECT "/Behaviour"
#define PERCEPTION PROJECT "/Perception"

AcceuillirClient::AcceuillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> perceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager) : Motivation(desireSet), m_StateManager(stateManager), m_PerceptionList(perceptionList)
{
    m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopicList.begin()->first, 10, &AcceuillirClient::VisionSubscriberCallBack, this));
}

void AcceuillirClient::VisionSubscriberCallBack(const homodeus_msgs::ObjectDetection& detected)
{
    if(detected.header.frame_id.find("Person"))
    {
        if(true || detected.header.frame_id.find("Entrée"))
        {
            m_PerceptionList[0] = true;
            VerifyCondition();
        }
    }
}

void AcceuillirClient::VerifyCondition()
{
    for(bool perception : m_PerceptionList)
    {
        if(!perception) return;
    }
    StateMachine();
}

void AcceuillirClient::StateMachine()
{
    m_StateManager->switchTo<GoToAccueilState>();
    
    for (bool&& perception : m_PerceptionList)
    {
        ROS_INFO_STREAM("Perception was : " << m_PerceptionList[0]);
        perception = false;
        ROS_INFO_STREAM("Perception now is  : " << m_PerceptionList[0]);
    }
}

PrendreCommande::PrendreCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> perceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager) : Motivation(desireSet) , m_PerceptionList(perceptionList)
{
    m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopicList.begin()->first, 10, &PrendreCommande::TimerSubscriberCallBack, this));
}

void PrendreCommande::TimerSubscriberCallBack(const std_msgs::Time time)
{
    double diffTime = ros::Time::now().toSec() - TIMEAUQUELTABLEAETEPRISE;
    if(diffTime > 180)
    {
        m_PerceptionList[0] = true;
        VerifyCondition();
    }
}

void PrendreCommande::VerifyCondition()
{
    for(bool perception : m_PerceptionList)
    {
        if (!perception) return;
    }
    StateMachine();
}

void PrendreCommande::StateMachine(){} // TODO : Add stuff

std::unique_ptr<Motivation> createAcceuillirMotivation(ros::NodeHandle& nodeHandle,std::shared_ptr<DesireSet> desireSet, StateManager* stateManager)
{
    return std::make_unique<AcceuillirClient>(std::map<std::string, bool>{{PERCEPTION "/Detect", false}}, nodeHandle, std::vector<bool>{false}, desireSet, stateManager);
}