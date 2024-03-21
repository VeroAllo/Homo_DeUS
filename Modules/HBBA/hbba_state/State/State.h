#ifndef HD_STATE_H
#define HD_STATE_H

#include <hbba_lite/core/DesireSet.h>
#include <hbba_lite/utils/ClassMacros.h>

#include <ros/ros.h>

#include <memory>
#include <numeric>
#include <vector>
#include <typeindex>

constexpr double TIMEOUT_S = 30;
constexpr uint64_t MAX_DESIRE_ID = std::numeric_limits<uint64_t>::max();  // TODO change to optional with C++17
constexpr int FLOAT_NUMBER_PRECISION = 3;

class StateManager;

class State
{
    bool m_enabled;
    std::type_index m_previousStageType;

protected:
    StateManager& m_stateManager;
    std::shared_ptr<DesireSet> m_desireSet;
    ros::NodeHandle& m_nodeHandle;

    std::vector<uint64_t> m_desireIds;

public:
    State(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle);
    virtual ~State() = default;

    DECLARE_NOT_COPYABLE(State);
    DECLARE_NOT_MOVABLE(State);

protected:
    bool enabled() const;
    std::type_index previousStageType() const;
    virtual std::type_index type() const = 0;

    virtual void enable(const std::string& parameter, const std::type_index& previousStageType);
    virtual void disable();

    friend StateManager;
};

inline bool State::enabled() const
{
    return m_enabled;
}

inline std::type_index State::previousStageType() const
{
    return m_previousStageType;
}

#endif