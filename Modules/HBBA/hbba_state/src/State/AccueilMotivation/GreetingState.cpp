#include "GreetingState.h"
#include "../StateManager.h"
#include "../commons/GoToTableState.h"

using namespace std;

GreetingState::GreetingState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle)
    : TalkState(stateManager, desireSet, nodeHandle, type_index(typeid(GoToTableState)))
{

}

GreetingState::~GreetingState(){
    
}

string GreetingState::generateText(const string& parameter){
    return "Greeting, would you please follow me.";
}
    

