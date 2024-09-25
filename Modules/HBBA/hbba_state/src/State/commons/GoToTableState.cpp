#include "GoToTableState.h"
#include "../StateManager.h"

using namespace std;

GoToTableState::GoToTableState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType,
    int Intensite)
    : GoToState(stateManager, desireSet, nodeHandle, nextStateType, Intensite)
{
    ROS_INFO("state GoToTableState creer");
}

GoToTableState::~GoToTableState() {}

string GoToTableState::generateGoal(const std::string& parameter)
{
    return parameter;
}