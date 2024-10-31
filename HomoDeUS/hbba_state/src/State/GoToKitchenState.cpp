#include "GoToKitchenState.h"
#include "StateManager.h"

using namespace std;

GoToKitchenState::GoToKitchenState(
StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType,
    int Intensite)
    : GoToState(stateManager, desireSet, nodeHandle, nextStateType, Intensite)
{

}

GoToKitchenState::~GoToKitchenState(){}

void GoToKitchenState::enable(const std::string& parameter, const std::type_index& previousStageType) 
{
    ROS_INFO("Started GotoKitchenState");
    GoToState::enable(parameter, previousStageType);
}


string GoToKitchenState::generateGoal(const std::string& parameter)
{
    return "Kitchen";
}