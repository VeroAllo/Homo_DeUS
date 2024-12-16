#include "GoToAccueilState.h"
#include "../StateManager.h"
#include "GreetingState.h"

using namespace std;

GoToAccueilState::GoToAccueilState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType,
    int Intensite)
    : GoToState(stateManager, desireSet, nodeHandle,  type_index(typeid(GreetingState)), Intensite)
{
    ROS_INFO("state GoToAccueilState creer");
}

GoToAccueilState::~GoToAccueilState(){}

string GoToAccueilState::generateGoal(const std::string& parameter)
{
    return "Accueil";
}