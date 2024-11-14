#ifndef HD_GOODBYE_STATE_H
#define HD_GOODBYE_STATE_H

#include "TalkState.h"

class GoodbyeState : public TalkState
{
public :
    GoodbyeState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType,
        int Intensite, std::string language);
    ~GoodbyeState() override;

    DECLARE_NOT_COPYABLE(GoodbyeState);
    DECLARE_NOT_MOVABLE(GoodbyeState);

protected:
    std::type_index type() const override;

    std::string generateText(const std::string& parameter) override;
   
};

inline std::type_index GoodbyeState::type() const
{
    return std::type_index(typeid(GoodbyeState));
}

#endif