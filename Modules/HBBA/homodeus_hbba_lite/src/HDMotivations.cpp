#include <homodeus_hbba_lite/HDMotivations.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>

#define TIMEAUQUELTABLEAETEPRISE 1

AcceuillirClient::AcceuillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, std::vector<Desire> desireList) : HDMotivation(desireSet, move(desireList)) 
{
    m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopicList.begin()->first, 10, &AcceuillirClient::VisionSubscriberCallBack, this));
    m_PerceptionList = PerceptionList;
}

void AcceuillirClient::VisionSubscriberCallBack(const std_msgs::String& msg)
{
    if(msg.data.find("Person"))
    {
        if(msg.data.find("Entrée"))
        {
            m_PerceptionList[0] = true;
            VerifyCondition();
        }
    }
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
    //To Do
}

PrendreCommande::PrendreCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, std::vector<Desire> desireList) : HDMotivation(desireSet, move(desireList)) 
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

ChercherCommande::ChercherCommande (const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, std::vector<Desire> desireList) : HDMotivation(desireSet, move(desireList)) 
{
    m_PerceptionList = PerceptionList;
}