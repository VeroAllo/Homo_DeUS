#include "AccueilMotiv_StateManager.h"
#include "../commons/GoToTableState.h"
#include "GreetingState.h"
#include "GoToAccueilState.h"


#include "../../hbba_core/HDStrategies.h"

using namespace std;

AccueilStateManager::AccueilStateManager(
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle, 
    vector<unique_ptr<BaseStrategy>> strategies, 
    shared_ptr<FilterPool> filterPool)
    :
    m_desireSet(move(desireSet)),
    m_nodeHandle(nodeHandle)
{
    setup(strategies, filterPool);
    type_index greetingStateType = std::type_index(typeid(GreetingState));

    addState(make_unique<GoToAccueilState>(this, desireSet, nodeHandle));
    addState(make_unique<GreetingState>(this, desireSet, nodeHandle));
    addState(
        make_unique<GoToTableState>(this, desireSet, nodeHandle, greetingStateType)
    );

}
AccueilStateManager::~AccueilStateManager(){

}

void AccueilStateManager::setup(vector<unique_ptr<BaseStrategy>> strategies, shared_ptr<FilterPool> filterPool){

    //setup strategy needed for state
    // strategies.emplace_back(createSpeechToTextStrategy(filterPool));
    // strategies.emplace_back(createExploreStrategy(filterPool));
    // strategies.emplace_back(createTalkStrategy(filterPool, m_desireSet, m_nodeHandle));
    // strategies.emplace_back(createGoToStrategy(filterPool));

}