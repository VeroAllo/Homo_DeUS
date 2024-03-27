#include "GoToAccueilState.h"
#include "../StateManager.h"
#include "GreetingState.h"

using namespace std;

GoToAccueilState::GoToAccueilState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle)
    : GoToState(stateManager, desireSet, nodeHandle,  type_index(typeid(GreetingState)))
{
    ROS_INFO("state GoToAccueilState creer");
}

GoToAccueilState::~GoToAccueilState(){}

string GoToAccueilState::generateGoal()
{
    return "Accueil";
}