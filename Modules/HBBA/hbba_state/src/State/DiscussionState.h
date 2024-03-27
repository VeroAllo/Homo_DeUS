#ifndef HD_DISCUSSION_STATE_H
#define HD_DISCUSSION_STATE_H

#include "State.h"

class DiscussionState : public State, public DesireSetObserver
{
    std::type_index m_nextStateType;
    uint64_t m_discussDesireId;

    public:
    DiscussionState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType);
    ~DiscussionState() override;

    DECLARE_NOT_COPYABLE(DiscussionState);
    DECLARE_NOT_MOVABLE(DiscussionState);

     void onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& _) override;

protected:
    void enable(const std::string& parameter, const std::type_index& previousStageType) override;
    void disable() override;

private:
    virtual std::string generateDiscussion();
};
#endif