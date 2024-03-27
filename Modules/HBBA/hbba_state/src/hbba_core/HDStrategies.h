#include "HDStrategy.h"
#include "HDDesires.h"

class GotoStrategy : public HDStrategy<GotoDesire>
{
    std::string m_desireID;
    public:
        GotoStrategy(std::shared_ptr<FilterPool> filterPool);
        void Publish(GotoDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};

class TalkStrategy : public HDStrategy<TalkDesire>
{
    std::string m_desireID;
    public:
        TalkStrategy();
        void Publish(TalkDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};

class DiscussStrategy : public HDStrategy<DiscussDesire>
{
    std::string m_desireID;
    public:
        DiscussStrategy();
        void Publish(DiscussDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};

class TakeStrategy : public HDStrategy<TakeDesire>
{
    std::string m_desireID;
    public:
        TakeStrategy();
        void Publish(TakeDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};

class DropStrategy : public HDStrategy<DropDesire>
{
    std::string m_desireID;
    public:
        DropStrategy();
        void Publish(DropDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};

class ExploreStrategy : public HDStrategy<ExploreDesire>
{
    std::string m_desireID;
    public:
        ExploreStrategy();
        void Publish(ExploreDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};
