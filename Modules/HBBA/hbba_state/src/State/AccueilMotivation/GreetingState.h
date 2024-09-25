#ifndef HD_GREETING_STATE_H
#define HD_GREETING_STATE_H

#include "../TalkState.h"

class GreetingState : public TalkState
{
public :
    GreetingState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType,
        int Intensite);
    ~GreetingState() override;

    DECLARE_NOT_COPYABLE(GreetingState);
    DECLARE_NOT_MOVABLE(GreetingState);

protected:
    std::type_index type() const override;

    std::string generateText(const std::string& parameter) override;
   
};

inline std::type_index GreetingState::type() const
{
    return std::type_index(typeid(GreetingState));
}

#endif