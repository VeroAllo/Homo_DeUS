#include "GreetingState.h"
#include "StateManager.h"

//#include <t_top_hbba_lite/Desires.h>

using namespace std;

GreetingState::GreetingState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    type_index nextStateType)
    : TalkState(stateManager, desireSet, nodeHandle, type_index(typeid(GreetingState)))
{

}
string GreetingState::generateText(const std::string& parameter){
    return "Greeting, would you please follow me.";
}
    

