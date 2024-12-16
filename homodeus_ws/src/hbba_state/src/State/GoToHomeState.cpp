#include "GoToHomeState.h"
#include "StateManager.h"

using namespace std;

GoToHomeState::GoToHomeState(
StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType,
    int Intensite)
    : GoToState(stateManager, desireSet, nodeHandle, nextStateType, Intensite)
{

}

GoToHomeState::~GoToHomeState(){}

void GoToHomeState::enable(const std::string& parameter, const std::type_index& previousStageType) 
{
    ROS_INFO("Started GoToHomeState");
    GoToState::enable(parameter, previousStageType);
}


string GoToHomeState::generateGoal(const std::string& parameter)
{
    return "Home";
}