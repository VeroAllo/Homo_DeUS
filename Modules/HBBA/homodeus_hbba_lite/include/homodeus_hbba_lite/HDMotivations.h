#include <ros/ros.h>
#include<homodeus_hbba_lite/HDMotivation.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>

class AcceuillirClient : public HDMotivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
public:
    AcceuillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, std::vector<Desire> desireList);
    void VisionSubscriberCallBack(const std_msgs::String& msg);
    void VerifyCondition();
    void StateMachine();
};

class PrendreCommande : public HDMotivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
public:
    PrendreCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, std::vector<Desire> desireList);
    void TimerSubscriberCallBack(const std_msgs::Time time);
    void VerifyCondition();
    void StateMachine();
};

class ChercherCommande : public HDMotivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
public:
    ChercherCommande(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList, std::shared_ptr<DesireSet> desireSet, std::vector<Desire> desireList);
    void VerifyCondition();
    void StateMachine();
};