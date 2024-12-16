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
    ROS_INFO_STREAM(parameter);
    if(parameter.find("pomme") != std::string::npos)
        return "Kitchen_pomme";
    else if(parameter.find("orange") != std::string::npos)
        return "Kitchen_orange";
    else
        return "Kitchen_fruit";
}