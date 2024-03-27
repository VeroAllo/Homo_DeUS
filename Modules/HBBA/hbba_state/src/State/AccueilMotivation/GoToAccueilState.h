#ifndef HD_GOTO_ACCUEIL_STATE_H
#define HD_GOTO_ACCUEIL_STATE_H
#include "../GoToState.h"

class GoToAccueilState : public GoToState
{
public:
    GoToAccueilState(StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle);

    ~GoToAccueilState() override;

    DECLARE_NOT_COPYABLE(GoToAccueilState);
    DECLARE_NOT_MOVABLE(GoToAccueilState);

protected:
    std::type_index type() const override;

    std::string generateGoal() override;
   
};
inline std::type_index GoToAccueilState::type() const
{
    return std::type_index(typeid(GoToAccueilState));
}

#endif