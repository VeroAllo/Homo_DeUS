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


string GoToKitchenState::generateGoal(const std::string& parameter)
{
    return "Kitchen";
}