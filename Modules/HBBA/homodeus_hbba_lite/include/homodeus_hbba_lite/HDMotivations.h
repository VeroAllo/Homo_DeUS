#include <ros/ros.h>
#include<hbba_lite/include/hbba_lite/core/Motivation.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>

class AcceuillirClient : public Motivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
public:
    AcceuillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet);
    void VisionSubscriberCallBack(const std_msgs::String& msg);
    void VerifyCondition();
    void StateMachine();
};

class PrendreCommande : public Motivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
public:
    PrendreCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet);
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

std::unique_ptr<HDMotivation> createAcceuillirMotivation(ros::NodeHandle& nodeHandle, std::shared_ptr<DesireSet> desireSet);