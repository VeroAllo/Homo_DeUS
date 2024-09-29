#include <homodeus_hbba_lite/HDMotivations.h>
#include <homodeus_msgs/ObjectDetection.h>
#include <homodeus_msgs/ObjectsDetection.h>
#include <../../hbba_state/src/State/TakeState.h>
#include "HDMotivations.h"
#include "HDStateIndexManager.h"
#include "GoToAccueilState.h"
#include "GoToTableState.h"
#include "GoToKitchenState.h"
#include "TakeState.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <memory>
#include <vector>
#include <map>
#include <../../hbba_state/src/State/GoToKitchenState.h>

#define TEST true
#define TEMPSDATTENTE 10.0f
#define NBTABLES 4
#define PROJECT "/Homodeus"
#define BEHAVIOUR PROJECT "/Behaviour"
#define PERCEPTION PROJECT "/Perception"

AccueillirClient::AccueillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> perceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager) : Motivation(desireSet), m_StateManager(stateManager), m_PerceptionList(perceptionList), strategy_motivation_interface_(nodeHandle)
{
    this->desireSet = desireSet;
    this->nodeHandle = nodeHandle;
    this->m_StateManager = stateManager;
    m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopicList.begin()->first, 10, &AccueillirClient::VisionSubscriberCallBack, this));
    strategy_motivation_interface_.setCallback(std::bind(&AccueillirClient::StrategySubscriberCallBack, this, std::placeholders::_1));
}

void AccueillirClient::VisionSubscriberCallBack(const homodeus_msgs::ObjectsDetection& detected)
{
    bool Person = false;
    for (const homodeus_msgs::ObjectDetection& detected_object : detected.objects)
    {
        if(detected_object.header.frame_id.find("person") != std::string::npos)
        {
            Person = true;
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

void AccueillirClient::StrategySubscriberCallBack(const std_msgs::String& msg)
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
    int index = StateIndexManager::getInstance().getNextIndex();
    std::type_index idleStateType = type_index(typeid(IdleState));
    std::type_index gotoAccueilStateType = type_index(typeid(GoToAccueilState));
    std::type_index gotoTableStateType = type_index(typeid(GoToTableState));
    std::type_index greetingStateType = type_index(typeid(GreetingState));
    std::vector<std::pair<int, std::unique_ptr<State>>> statesToCreate = {
        {index, std::make_unique<IdleState>(this->m_StateManager, this->desireSet, this->nodeHandle, gotoAccueilStateType, 0)},
        {index, std::make_unique<GoToAccueilState>(this->m_StateManager, this->desireSet, this->nodeHandle, greetingStateType, 1)},
        {index, std::make_unique<GreetingState>(this->m_StateManager, this->desireSet, this->nodeHandle, greetingStateType, 2)},
        {index, std::make_unique<GoToTableState>(this->m_StateManager, this->desireSet, this->nodeHandle, idleStateType, 3)}
    };

    for (auto& statePair : statesToCreate)
    {
        m_StateManager->addState(statePair.first, std::move(statePair.second));
    }

    m_StateManager->switchTo<GoToAccueilState>(index);

    for (bool&& perception : m_PerceptionList)
    {
        ROS_INFO_STREAM("Perception was : " << m_PerceptionList[0]);
        perception = false;
        ROS_INFO_STREAM("Perception now is  : " << m_PerceptionList[0]);
    }
}

PrendreCommande::PrendreCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> perceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager) : Motivation(desireSet), m_StateManager(stateManager), m_PerceptionList(perceptionList), strategy_motivation_interface_(nodeHandle)
{
    this->desireSet = desireSet;
    this->nodeHandle = nodeHandle;
    this->m_StateManager = stateManager;
    m_Timers.reserve(10);
    strategy_motivation_interface_.setCallback(std::bind(&PrendreCommande::StrategySubscriberCallBack, this, std::placeholders::_1));    
    m_Timers[0] = nodeHandle.createTimer(ros::Duration(TEMPSDATTENTE), [this](const ros::TimerEvent&) { this->TimerSubscriberCallBack(0); }, false, false);
    m_Timers[1] = nodeHandle.createTimer(ros::Duration(TEMPSDATTENTE), [this](const ros::TimerEvent&) { this->TimerSubscriberCallBack(1); }, false, false);
    m_Timers[2] = nodeHandle.createTimer(ros::Duration(TEMPSDATTENTE), [this](const ros::TimerEvent&) { this->TimerSubscriberCallBack(2); }, false, false);
    m_Timers[3] = nodeHandle.createTimer(ros::Duration(TEMPSDATTENTE), [this](const ros::TimerEvent&) { this->TimerSubscriberCallBack(3); }, false, false);
}

void PrendreCommande::StrategySubscriberCallBack(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("Received message from GotoStrategy: " << msg.data);
    if (msg.data.find("Table") != std::string::npos)
    {
        int table = std::stoi(msg.data.substr(5,6));
        if (m_Tables[table] = false)
        {
            m_Tables[table] = true;
            m_Timers[table].start();
        }
    }
}

void PrendreCommande::TimerSubscriberCallBack(int table)
{
    m_Timers[table].stop();
    VerifyCondition(table);
}

void PrendreCommande::VerifyCondition(int tb)
{
    if(m_Tables[tb] == true || TEST)
    {
        StateMachine(tb);
    }
}

void PrendreCommande::StateMachine(int tb)
{
    int index = StateIndexManager::getInstance().getNextIndex();
    std::type_index idleStateType = type_index(typeid(IdleState));
    std::type_index discussStateType = type_index(typeid(DiscussionState));
    std::type_index gotoTableStateType = type_index(typeid(GoToTableState));
    std::vector<std::pair<int, std::unique_ptr<State>>> statesToCreate = {
        {index, std::make_unique<IdleState>(this->m_StateManager, this->desireSet, this->nodeHandle, gotoTableStateType, 0)},
        {index, std::make_unique<GoToTableState>(this->m_StateManager, this->desireSet, this->nodeHandle, discussStateType, 1)},
        {index, std::make_unique<DiscussionState>(this->m_StateManager, this->desireSet, this->nodeHandle, idleStateType, 2)}

    for (auto& statePair : statesToCreate)
    {
        m_StateManager->addState(statePair.first, std::move(statePair.second));
    }

    // Switch to the desired state
    m_StateManager->switchTo<GoToTableState>(index, "Table" + std::to_string(tb));
}

ChercherCommande::ChercherCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> perceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager) : Motivation(desireSet), m_StateManager(stateManager), m_PerceptionList(perceptionList), strategy_motivation_interface_(nodeHandle)
{
    this->desireSet = desireSet;
    this->nodeHandle = nodeHandle;
    this->m_StateManager = stateManager;
    strategy_motivation_interface_.setCallback(std::bind(&ChercherCommande::StrategySubscriberCallBack, this, std::placeholders::_1));
}

void ChercherCommande::StrategySubscriberCallBack(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("Received message from Discuss: " << msg.data);
    if (msg.data.find("Commande:") != std::string::npos)
    {
        static_cast<TakeState*>(m_StateManager->m_listsStates[2][std::type_index(typeid(TakeState))].get())->GenerateObjectToTake(msg.data.substr(9));
        StateMachine("test");
    }
}

void ChercherCommande::StateMachine(std::string commande)
{
    int index = StateIndexManager::getInstance().getNextIndex();
    std::type_index idleStateType = type_index(typeid(IdleState));
    std::type_index gotoKitchenStateType = type_index(typeid(GoToKitchenState));
    std::type_index gotoTableStateType = type_index(typeid(GoToTableState));
    std::type_index takeStateType = type_index(typeid(TakeState));
    std::vector<std::pair<int, std::unique_ptr<State>>> statesToCreate = {
        {index, std::make_unique<IdleState>(this->m_StateManager, this->desireSet, this->nodeHandle, gotoKitchenStateType, 0)},
        {index, std::make_unique<GoToKitchenState>(this->m_StateManager, this->desireSet, this->nodeHandle, takeStateType, 1)},
        {index, std::make_unique<TakeState>(this->m_StateManager, this->desireSet, this->nodeHandle, gotoTableStateType, 2)},
        {index, std::make_unique<GoToTableState>(this->m_StateManager, this->desireSet, this->nodeHandle, idleStateType, 3)},
    };

    for (auto& statePair : statesToCreate)
    {
        m_StateManager->addState(statePair.first, std::move(statePair.second));
    }

    m_StateManager->switchTo<GoToKitchenState>(index, commande);
}

std::unique_ptr<Motivation> createAccueillirMotivation(ros::NodeHandle& nodeHandle,std::shared_ptr<DesireSet> desireSet, StateManager* stateManager)
{
    return std::make_unique<AcceuillirClient>(std::map<std::string, bool>{{PERCEPTION "/Detect", false}}, nodeHandle, std::vector<bool>{false}, desireSet, stateManager);
}

std::unique_ptr<Motivation> createPrendreCommande(ros::NodeHandle& nodeHandle,std::shared_ptr<DesireSet> desireSet, StateManager* stateManager)
{
    return std::make_unique<PrendreCommande>(std::map<std::string, bool>{}, nodeHandle, std::vector<bool>{false}, desireSet, stateManager);
}

std::unique_ptr<Motivation> createChercherCommande(ros::NodeHandle& nodeHandle,std::shared_ptr<DesireSet> desireSet, StateManager* stateManager)
{
    return std::make_unique<ChercherCommande>(std::map<std::string, bool>{}, nodeHandle, std::vector<bool>{false}, desireSet, stateManager);
}