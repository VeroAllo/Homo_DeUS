#include <ros/ros.h>
#include <hbba_lite/core/Motivation.h>
#include <../../hbba_state/src/State/StateManager.h>
#include <../../hbba_state/src/State/AccueilMotivation/GoToAccueilState.h>
#include <std_msgs/String.h>
#include <homodeus_msgs/ObjectDetection.h>
#include <homodeus_msgs/ObjectsDetection.h>
#include <std_msgs/Time.h>

class AcceuillirClient : public Motivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
    StateManager* m_StateManager;
public:
    AcceuillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager);
    void VisionSubscriberCallBack(const homodeus_msgs::ObjectsDetection& detected);
    void VerifyCondition();
    void StateMachine();
};

class PrendreCommande : public Motivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
public:
    PrendreCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager);
    void TimerSubscriberCallBack(const std_msgs::Time time);
    void VerifyCondition();
    void StateMachine();
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

std::unique_ptr<Motivation> createAcceuillirMotivation(ros::NodeHandle& nodeHandle, std::shared_ptr<DesireSet> desireSet, StateManager* stateManager);