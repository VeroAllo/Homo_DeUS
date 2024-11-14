#include "StateManager.h"
#include "commons/GoToTableState.h"
#include "GoodbyeState.h"

using namespace std;

GoodbyeState::GoodbyeState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType,
    int Intensite, std::string language)
    : TalkState(stateManager, desireSet, nodeHandle, nextStateType, Intensite, language)
{

}

GoodbyeState::~GoodbyeState(){
    
}

string GoodbyeState::generateText(const string& parameter){
    if (m_language == "en"){
        return "Goodbye";
    }
    else {
        return "Au revoir";
    }
    
}
