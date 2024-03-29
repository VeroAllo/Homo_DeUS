#include "AccueilMotiv_StateManager.h"
#include <homodeus_hbba_lite/HDDesires.h>
#include <homodeus_hbba_lite/HDStrategies.h>
#include "../commons/GoToTableState.h"
#include "GreetingState.h"
#include "GoToAccueilState.h"



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
    setup(move(strategies), move(filterPool));
    type_index greetingStateType = std::type_index(typeid(GreetingState));
    ROS_INFO("dans Accueil state manager");
    // addState(make_unique<GoToAccueilState>(this, desireSet, nodeHandle));
    // addState(make_unique<GreetingState>(this, desireSet, nodeHandle));
    stateManager.addState(
        make_unique<GoToTableState>(stateManager, desireSet, nodeHandle, greetingStateType)
    );
    ROS_INFO("state gotoTableState fait");

}
AccueilStateManager::~AccueilStateManager(){

}

void AccueilStateManager::setup(vector<unique_ptr<BaseStrategy>> strategies, shared_ptr<FilterPool> filterPool){

    //setup strategy needed for state
    //strategies.emplace_back(createSpeechToTextStrategy(filterPool)); does not exist lol
    strategies.emplace_back(createExploreStrategy(filterPool, m_desireSet, m_nodeHandle));
    strategies.emplace_back(createDiscussStrategy(filterPool, m_desireSet, m_nodeHandle,1));
    strategies.emplace_back(createGoToStrategy(filterPool, m_desireSet, m_nodeHandle,1));

}