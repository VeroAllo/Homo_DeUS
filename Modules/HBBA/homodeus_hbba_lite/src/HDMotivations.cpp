#include <homodeus_hbba_lite/HDMotivations.h>
#include <homodeus_msgs/ObjectDetection.h>
#include <homodeus_msgs/ObjectsDetection.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>

#define TIMEAUQUELTABLEAETEPRISE 1
#define PROJECT "/Homodeus"
#define BEHAVIOUR PROJECT "/Behaviour"
#define PERCEPTION PROJECT "/Perception"

AccueillirClient::AccueillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> perceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager) : Motivation(desireSet), m_StateManager(stateManager), m_PerceptionList(perceptionList), strategy_motivation_interface_(nodeHandle)
{
    m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopicList.begin()->first, 10, &AccueillirClient::VisionSubscriberCallBack, this));
    strategy_motivation_interface_.setCallback(std::bind(&AccueillirClient::StrategySubscriberCallBack, this, std::placeholders::_1));
}

void AccueillirClient::VisionSubscriberCallBack(const homodeus_msgs::ObjectsDetection& detected)
{
    Person = false;
    for (const homodeus_msgs::ObjectDetection& detected_object : detected.objects)
    {
        if(detected_object.header.frame_id.find("person"))
        {
            Person = true
            if(TimeDelay > 2)
            {
                TimeDelay = 0;
                m_PerceptionList[0] = true;
                VerifyCondition();
                return;
            }
        }
    }
    if (Person = true)
    {
        // ajouter timer 
        TimeDelay++;
    } else {
        TimeDelay = 0;
    }
    
}

void AccueillirClient::StrategySubscriberCallBack(const homodeus_msgs::StrategyToMotivation& msg)
{
    ROS_INFO_STREAM("Received message from GotoStrategy: " << msg.data);
}

void AccueillirClient::VerifyCondition()
{
    for(bool perception : m_PerceptionList)
    {
        if(!perception) return;
    }
    StateMachine();
}

void AccueillirClient::StateMachine()
{

    m_StateManager->switchTo<GoToAccueilState>(0);
    for (bool&& perception : m_PerceptionList)
    {
        ROS_INFO_STREAM("Perception was : " << m_PerceptionList[0]);
        perception = false;
        ROS_INFO_STREAM("Perception now is  : " << m_PerceptionList[0]);
    }
}

PrendreCommande::PrendreCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> perceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager) : Motivation(desireSet) , m_PerceptionList(perceptionList), strategy_motivation_interface_(nodeHandle)
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

std::unique_ptr<Motivation> createAccueillirMotivation(ros::NodeHandle& nodeHandle,std::shared_ptr<DesireSet> desireSet, StateManager* stateManager)
{
    return std::make_unique<AcceuillirClient>(std::map<std::string, bool>{{PERCEPTION "/Detect", false}}, nodeHandle, std::vector<bool>{false}, desireSet, stateManager);
}