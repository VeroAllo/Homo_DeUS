#include "GoToTableState.h"
#include "StateManager.h"

//#include <t_top_hbba_lite/Desires.h>

using namespace std;

GoToTableState::GoToTableState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType)
    : GoToState(stateManager, desireSet, nodeHandle, nextStateType)
{

}

string generateGoal(const string& parameter)
{
    return "Table";
}