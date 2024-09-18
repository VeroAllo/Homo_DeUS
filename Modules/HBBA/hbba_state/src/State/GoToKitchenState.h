#ifndef HD_GOTO_KITCHEN_STATE_H
#define HD_GOTO_KITCHEN_STATE_H

#include "GoToState.h"

class GoToKitchenState : public GoToState
{
public:
    GoToKitchenState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType);

    ~GoToKitchenState() override;
    DECLARE_NOT_COPYABLE(GoToKitchenState);
    DECLARE_NOT_MOVABLE(GoToKitchenState);

protected:
    std::type_index type() const override;

    std::string generateGoal(const std::string& parameter = "") override;
   
};

inline std::type_index GoToKitchenState::type() const
{
    return std::type_index(typeid(GoToKitchenState));
}
#endif