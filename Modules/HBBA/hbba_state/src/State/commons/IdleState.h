#ifndef HD_IDLE_STATE_H
#define HD_IDLE_STATE_H

#include "../State.h"

class IdleState : public State
{
    //uint64_t m_idleDesireId;

public:
    IdleState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType);
    ~IdleState() override;

    DECLARE_NOT_COPYABLE(IdleState);
    DECLARE_NOT_MOVABLE(IdleState);

    //void onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& _) override;

protected:
    void enable(const std::string& parameter, const std::type_index& previousStageType) override;
    void disable() override;
    std::type_index type() const override;
};

inline std::type_index IdleState::type() const
{
    return std::type_index(typeid(IdleState));
}

#endif