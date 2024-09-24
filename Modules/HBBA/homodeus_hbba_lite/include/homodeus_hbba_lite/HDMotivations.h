#include <ros/ros.h>
#include <hbba_lite/core/Motivation.h>
#include <../../hbba_state/src/State/StateManager.h>
#include <../../hbba_state/src/State/AccueilMotivation/GoToAccueilState.h>
#include <std_msgs/String.h>
#include <homodeus_msgs/ObjectDetection.h>
#include <homodeus_msgs/ObjectsDetection.h>
#include <std_msgs/Time.h>
#include "HDStrategyToMotivationInterface.h"
#include <time.h>
#include <../../hbba_state/src/State/commons/GoToTableState.h>

class AccueillirClient : public Motivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
    StateManager* m_StateManager;
    int TimeDelay = 0;
public:
    AccueillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager);
    void VisionSubscriberCallBack(const homodeus_msgs::ObjectsDetection& detected);
    void StrategySubscriberCallBack(const std_msgs::String& msg);
    void VerifyCondition();
    void StateMachine();
    HDStrategyMotivationInterface strategy_motivation_interface_;
};

class PrendreCommande : public Motivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
    StateManager* m_StateManager;
    std::vector<bool> m_Tables{false, false, false, false};
    std::vector<ros::Timer> m_Timers;
public:
    PrendreCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager);
    void TimerSubscriberCallBack(int table);
    void StrategySubscriberCallBack(const std_msgs::String& msg);
    void VerifyCondition(int table);
    void StateMachine(int tb);
    HDStrategyMotivationInterface strategy_motivation_interface_;
};

class ChercherCommande : public Motivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
public:
    ChercherCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet);
    void VerifyCondition();
    void StateMachine();
};

std::unique_ptr<Motivation> createAccueillirMotivation(ros::NodeHandle& nodeHandle, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager);

std::unique_ptr<Motivation> createPrendreCommandeMotivation(ros::NodeHandle& nodeHandle, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager);