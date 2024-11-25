#include "HDDesires.h"
#include "HDStrategy.h"
#include <hbba_lite/core/Strategy.h> //Pour les fucking constructeur de FilterConfiguration
#include <ros/ros.h>
#include <vector>
#include <memory>
#include <../../hbba_state/src/State/StateManager.h>
#include "HDStrategyToMotivationInterface.h"

class GotoStrategy : public HDStrategy<GotoDesire>
{
    public:
        GotoStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void onEnabling(const GotoDesire& desire) override;
        void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) override;
        void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) override;
        void SubscriberStatusCallBack(const homodeus_msgs::HDPose& hdPose);
        void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) override {}
    private:
        HDStrategyMotivationInterface strategy_motivation_interface_;
};

class TalkStrategy : public HDStrategy<TalkDesire>
{
    public:
        TalkStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void onEnabling(const TalkDesire& desire) override;
        void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) override;
        void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) override;
        void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) override;
    private:
        HDStrategyMotivationInterface strategy_motivation_interface_;
};

class DiscussStrategy : public HDStrategy<DiscussDesire>
{
    public:
        DiscussStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);
        void onEnabling(const DiscussDesire& desire) override;
        void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) override;
        void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) override;
        void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) override;
    private:
        HDStrategyMotivationInterface strategy_motivation_interface_;
};

class TakeStrategy : public HDStrategy<TakeDesire>
{
    public:
        TakeStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, std::map<std::string,bool> publisherTopicList, std::map<std::string,bool> subscriberTopicList, std::shared_ptr<DesireSet> desireSet,std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName, StateManager* stateManager);
        void onEnabling(const TakeDesire& desire) override;
        void SubscriberResponseCallBack(const homodeus_msgs::HDResponse& response) override;
        void SubscriberCancelCallBack(const homodeus_msgs::DesireID& desireID) override;
        void SubscriberStatusCallBack(const homodeus_msgs::HDResponse& response);
        void SubscriberStatusCallBack(const homodeus_msgs::HDStatus& status) override {}
        void SubscriberVisionCallback(const homodeus_msgs::ObjectsDetection& status) override;
    private:
        homodeus_msgs::ObjectDetection GetClosestTagMatchingCommande(const std::string& commande)
        {
            homodeus_msgs::ObjectsDetection objectsToDetectCopy = m_ObjectsToDetect;

            ROS_INFO_STREAM(commande);
            ROS_INFO_STREAM(objectsToDetectCopy.objects.size());
            ROS_INFO_STREAM(m_ObjectsToDetect.objects.size());
            std::remove_if(objectsToDetectCopy.objects.begin(), objectsToDetectCopy.objects.end(), [commande](const homodeus_msgs::ObjectDetection& object){ return object.header.frame_id != commande; });
            ROS_INFO_STREAM(objectsToDetectCopy.objects.size());
            ROS_INFO_STREAM(m_ObjectsToDetect.objects.size());
            std::sort(objectsToDetectCopy.objects.begin(), objectsToDetectCopy.objects.end(), [](const homodeus_msgs::ObjectDetection& lhs, const homodeus_msgs::ObjectDetection& rhs) { return lhs.distance < rhs.distance; });
            if (objectsToDetectCopy.objects.size())
            {
                for (size_t i = 0; i < objectsToDetectCopy.objects.size(); i++)
                {
                    if (objectsToDetectCopy.objects[i].header.frame_id == commande)
                    {
                        m_ObjectsToDetect.objects.clear();
                        return objectsToDetectCopy.objects[i];
                    }                

                }

            m_ObjectsToDetect.objects.clear();
                homodeus_msgs::ObjectDetection temp = homodeus_msgs::ObjectDetection{};
                temp.header.frame_id = "NOT FOUND";
                return temp;
            }
            else
            {
            m_ObjectsToDetect.objects.clear();
                homodeus_msgs::ObjectDetection temp = homodeus_msgs::ObjectDetection{};
                temp.header.frame_id = "NOT FOUND";
                return temp;
            }
        }

        homodeus_msgs::ObjectsDetection m_ObjectsToDetect{};
        ros::NodeHandle& m_NodeHandle;
        StateManager* m_StateManager;
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

std::unique_ptr<BaseStrategy> createTakeStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, StateManager* stateManager,  uint16_t utility = 1U);

std::unique_ptr<BaseStrategy> createDropStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1U);

std::unique_ptr<BaseStrategy> createExploreStrategy(std::shared_ptr<FilterPool> filterPool, std::shared_ptr<DesireSet> desireSet, ros::NodeHandle& nodeHandle, uint16_t utility = 1U);
