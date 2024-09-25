#ifndef HD_DISCUSSION_STATE_H
#define HD_DISCUSSION_STATE_H

#include "State.h"

class DiscussionState : public State, public DesireSetObserver
{
    uint64_t m_discussDesireId;

    public:
    DiscussionState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType,
        std::int Intensite);
    ~DiscussionState() override;

    DECLARE_NOT_COPYABLE(DiscussionState);
    DECLARE_NOT_MOVABLE(DiscussionState);

     void onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& _) override;

protected:
    void enable(const std::string& parameter, const std::type_index& previousStageType) override;
    void disable() override;
    std::type_index type() const override;

private:
    virtual std::string generateDiscussion();
};


inline std::type_index DiscussionState::type() const
{
    return std::type_index(typeid(DiscussionState));
}

#endif