#include <homodeus_hbba_lite/HDMotivations.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>

#define TIMEAUQUELTABLEAETEPRISE 1
#define PROJECT "/Homodeus"
#define BEHAVIOUR PROJECT "/Behaviour"
#define PERCEPTION PROJECT "/Perception"

AcceuillirClient::AcceuillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* SM) : Motivation(desireSet) 
{
    m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopicList.begin()->first, 10, &AcceuillirClient::VisionSubscriberCallBack, this));
    m_PerceptionList = PerceptionList;
    m_StateManager = SM;
}

void AcceuillirClient::VisionSubscriberCallBack(const std_msgs::String& msg)
{
    m_PerceptionList[0] = true;
    VerifyCondition();
    /*if(msg.data.find("Person"))
    {
        if(msg.data.find("Entrée"))
        {
            m_PerceptionList[0] = true;
            VerifyCondition();
        }
    }
    */
}

void AcceuillirClient::VerifyCondition()
{
    bool valid = true;
    for(bool Perception : m_PerceptionList)
    {
        if(!Perception)
        {
            valid = false;
        } 
    }
    if(valid)
    {
        StateMachine();
    }
}

void AcceuillirClient::StateMachine()
{
    m_StateManager->switchTo<GoToAccueilState>();
}

PrendreCommande::PrendreCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet) : Motivation(desireSet) 
{
    m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopicList.begin()->first, 10, &PrendreCommande::TimerSubscriberCallBack, this));
    m_PerceptionList = PerceptionList;
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
    bool valid = true;
    for(bool Perception : m_PerceptionList)
    {
        if(!Perception)
        {
            valid = false;
        } 
    }
    if(valid)
    {
        StateMachine();
    }
}

void PrendreCommande::StateMachine()
{
    //To Do
}


std::unique_ptr<Motivation> createAcceuillirMotivation(ros::NodeHandle& nodeHandle,std::shared_ptr<DesireSet> desireSet, StateManager* SM)
{
    return std::make_unique<AcceuillirClient>(std::map<std::string, bool>{{PERCEPTION "/Detect", false}}, nodeHandle, std::vector<bool>{false}, desireSet, SM);
}