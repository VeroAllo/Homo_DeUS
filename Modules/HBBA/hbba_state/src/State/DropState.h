#ifndef HD_DROP_STATE_H
#define HD_DROP_STATE_H

#include "State.h"

class DropState : public State, public DesireSetObserver
{
    uint64_t m_dropDesireId;

public:
    DropState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType);
    ~DropState() override;

    DECLARE_NOT_COPYABLE(DropState);
    DECLARE_NOT_MOVABLE(DropState);

    void onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& _) override;

protected:
    void enable(const std::string& parameter, const std::type_index& previousStageType) override;
    void disable() override;
    std::type_index type() const override;
};

inline std::type_index DropState::type() const
{
    return std::type_index(typeid(DropState));
}

#endif

