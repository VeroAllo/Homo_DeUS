#include "IdleState.h"
#include "../StateManager.h"

#include <homodeus_hbba_lite/HDDesires.h>

using namespace std;

IdleState::IdleState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    type_index nextStateType,
    int intensite)
    : State(stateManager, desireSet, nodeHandle, nextStateType, intensite)
{
}

IdleState::~IdleState(){}

void IdleState::enable(const string& parameter, const type_index& previousStageType)
{
    State::enable(parameter, previousStageType);
}

void IdleState::disable()
{
    ROS_INFO("End of idle state");
    State::disable();
}
