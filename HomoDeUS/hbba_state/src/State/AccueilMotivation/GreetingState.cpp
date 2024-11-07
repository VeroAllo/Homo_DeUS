#include "GreetingState.h"
#include "../StateManager.h"
#include "../commons/GoToTableState.h"

using namespace std;

GreetingState::GreetingState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType,
    int Intensite)
    : TalkState(stateManager, desireSet, nodeHandle, type_index(typeid(GoToTableState)), Intensite)
{

}

GreetingState::~GreetingState(){
    
}

string GreetingState::generateText(const string& parameter){
    return "Greeting, would you please follow me.";
}
    

