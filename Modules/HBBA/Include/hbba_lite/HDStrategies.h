#include "HDDesires.h"
#include "HDStrategy.h"
#include <ros/ros.h>
#include <vector>
#include <memory>

class GotoStrategy : public HDStrategy<GotoDesire>
{
    public:
        GotoStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void Publish(const GotoDesire& desire);
        void SubscriberCallBack(const std_msgs::String& msg) override;
};

class TalkStrategy : public HDStrategy<SpeakDesire>
{
    public:
        TalkStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void Publish(const SpeakDesire& desire);
        void SubscriberCallBack(const std_msgs::String& msg) override;
};

class DiscussStrategy : public HDStrategy<DiscussDesire>
{
    public:
        DiscussStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void Publish(const DiscussDesire& desire);
        void SubscriberCallBack(const std_msgs::String& msg) override;
};

class TakeStrategy : public HDStrategy<TakeDesire>
{
    public:
        TakeStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void Publish(const TakeDesire& desire);
        void SubscriberCallBack(const std_msgs::String& msg) override;
};

class DropStrategy : public HDStrategy<DropDesire>
{
    public:
        DropStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void Publish(const DropDesire& desire);
        void SubscriberCallBack(const std_msgs::String& msg) override;
};

class ExploreStrategy : public HDStrategy<ExploreDesire>
{
    public:
        ExploreStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void Publish(const ExploreDesire& desire);
        void SubscriberCallBack(const std_msgs::String& msg) override;
};
