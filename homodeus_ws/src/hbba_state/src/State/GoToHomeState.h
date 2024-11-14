#ifndef HD_GOTO_HOME_STATE_H
#define HD_GOTO_HOME_STATE_H

#include "GoToState.h"

class GoToHomeState : public GoToState
{
public:
    GoToHomeState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType,
        int Intensite);

    ~GoToHomeState() override;
    DECLARE_NOT_COPYABLE(GoToHomeState);
    DECLARE_NOT_MOVABLE(GoToHomeState);

protected:
    std::type_index type() const override;

    void enable(const std::string& parameter, const std::type_index& previousStageType) override;

    std::string generateGoal(const std::string& parameter = "") override;
   
};

inline std::type_index GoToHomeState::type() const
{
    return std::type_index(typeid(GoToHomeState));
}
#endif