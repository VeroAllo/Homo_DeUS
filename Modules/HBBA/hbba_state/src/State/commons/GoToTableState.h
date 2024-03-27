#ifndef HD_GOTO_TABLE_STATE_H
#define HD_GOTO_TABLE_STATE_H

#include "../GoToState.h"

class GoToTableState : public GoToState
{
public:
    GoToTableState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType);
    ~GoToTableState() override;

    DECLARE_NOT_COPYABLE(GoToTableState);
    DECLARE_NOT_MOVABLE(GoToTableState);

protected:
    std::type_index type() const override;

    std::string generateGoal() override;
   
};

inline std::type_index GoToTableState::type() const
{
    return std::type_index(typeid(GoToTableState));
}

#endif