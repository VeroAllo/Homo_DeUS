#include "HDDesires.h"
#include "HDStrategy.h"
#include <hbba_lite/core/Strategy.h> //Pour les fucking constructeur de FilterConfiguration
#include <ros/ros.h>
#include <vector>
#include <memory>

class GotoStrategy : public HDStrategy<GotoDesire>
{
    public:
        GotoStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void onEnabling(const GotoDesire& desire) override;
        void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) override;
        void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) override;
        void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) override;
};

class TalkStrategy : public HDStrategy<TalkDesire>
{
    public:
        TalkStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void onEnabling(const TalkDesire& desire) override;
        void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) override;
        void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) override;
        void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) override;
};

class DiscussStrategy : public HDStrategy<DiscussDesire>
{
    public:
        DiscussStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void onEnabling(const DiscussDesire& desire) override;
        void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) override;
        void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) override;
        void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) override;
};

class TakeStrategy : public HDStrategy<TakeDesire>
{
    public:
        TakeStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void onEnabling(const TakeDesire& desire) override;
        void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) override;
        void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) override;
        void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) override;
};

class DropStrategy : public HDStrategy<DropDesire>
{
    public:
        DropStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void onEnabling(const DropDesire& desire) override;
        void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) override;
        void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) override;
        void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) override;
};

class ExploreStrategy : public HDStrategy<ExploreDesire>
{
    public:
        ExploreStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void onEnabling(const ExploreDesire& desire) override;
        void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) override;
        void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) override;
        void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) override;
};

std::unique_ptr<BaseStrategy> createGoToStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1U);

std::unique_ptr<BaseStrategy> createTalkStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1U);

std::unique_ptr<BaseStrategy> createDiscussStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1U);

std::unique_ptr<BaseStrategy> createTakeStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1U);

std::unique_ptr<BaseStrategy> createDropStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1U);

std::unique_ptr<BaseStrategy> createExploreStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1U);
