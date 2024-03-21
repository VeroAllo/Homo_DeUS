#ifndef HD_GOTO_STATE_H
#define HD_GOTO_STATE_H

#include "State.h"

class GoToState : public State, public DesireSetObserver
{
    std::type_index m_nextStateType;
    uint64_t m_gotoDesireId;

public:
    GoToState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType);
    ~GoToState() override;

    DECLARE_NOT_COPYABLE(GoToState);
    DECLARE_NOT_MOVABLE(GoToState);
    void onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& _) override;

protected:
    void enable(const std::string& parameter, const std::type_index& previousStageType) override;
    void disable() override;

private:
    virtual std::string generateGoal(const std::string& parameter) = 0;
};
#endif