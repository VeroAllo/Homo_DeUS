#include "GoToTableState.h"
#include "../StateManager.h"


using namespace std;

GoToTableState::GoToTableState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType)
    : GoToState(stateManager, desireSet, nodeHandle, nextStateType)
{

}
GoToTableState::~GoToTableState() {}

string GoToTableState::generateGoal()
{
    return "Table";
}