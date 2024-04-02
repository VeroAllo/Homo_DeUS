#include "StateManager.h"

using namespace std;

StateManager::StateManager(){
}

StateManager::~StateManager()
{
    State* tmp_state;
    for(auto a =  m_listStates.begin() ; a != m_listStates.end(); a++)
    {
        for(auto b = m_listStates[a->first].begin(); b != m_listStates[a->first].end(); b++)
        {
            tmp_state = b->second.get();
            if(tmp_state->enabled() == true){
                tmp_state->disable();
            }
        }
    }
}

void StateManager::addState(unique_ptr<State> state, int listId)
{
    m_listStates[listId].emplace(state->type(), move(state));
}

void StateManager::switchTo(State*currentState, std::type_index stateType, const std::string& parameter)
{
    std::type_index previousStageType(typeid(State));
    if(currentState != nullptr){
        currentState->disable();
        ROS_INFO("Disabling %s", currentState->type().name());
        previousStageType = currentState->type();
    }
    ROS_INFO("Enabling %s (%s)", stateType.name(), parameter.c_str());
    findState(currentState, stateType)->enable(parameter, previousStageType);
}

State* StateManager::findState(State* currentState, std::type_index stateType)
{
    State* tmp_state;
    for(int i=0; i < 3 ; i++)
    {
        tmp_state = m_listStates[i].at(stateType).get();
        if(tmp_state->nextStateType() == stateType){
            return tmp_state;
        }
    }
    ROS_INFO("State %s avec nextState %s n existe pas", currentState->type().name(), stateType.name());
    return tmp_state;
}

void StateManager::startStateMachine(int listKey)
{
}

// void StateManager::addListState(unique_ptr<State> firstState, int listId)
// {
//     std::unordered_map<std::type_index, std::unique_ptr<State>> tmp_states = {{firstState->type(), move(firstState)}};
//     m_listStates.emplace(listId, move(tmp_states));
// }