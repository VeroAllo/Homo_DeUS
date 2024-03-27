#include "GoToKitchenState.h"
#include "StateManager.h"

using namespace std;

GoToKitchenState::GoToKitchenState(
StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType)
    : GoToState(stateManager, desireSet, nodeHandle, nextStateType)
{

}

GoToKitchenState::~GoToKitchenState(){}


string GoToKitchenState::generateGoal()
{
    return "Kitchen";
}