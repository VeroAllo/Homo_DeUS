#include "StateManager.h"
#include "commons/IdleState.h"

using namespace std;

StateManager::StateManager() {}

StateManager::~StateManager()
{
    for(auto it = m_listsStates.begin(); it != m_listsStates.end() ; it++)
    {
        for(auto it2 = m_listsStates[it->first].begin(); it2 != m_listsStates[it->first].end(); it2++)
        {
            if(m_listsStates[it->first][it2->first].get()->enabled() == true)
                m_listsStates[it->first][it2->first].get()->disable();
        }
            
    }
}

void StateManager::addListStates(int indexList, unique_ptr<State> fistState)
{
    m_listsStates[indexList][fistState->type()] = move(fistState);
}

void StateManager::addState(int indexList, unique_ptr<State> state)
{
    m_listsStates[indexList][state->type()] = move(state);
}

void StateManager::switchTo(State* state, type_index stateType, const std::string& parameter)
{
    std::type_index nextStageType(typeid(State));

    nextStageType = stateType;
    State* tmp_state;
    for(auto& [key, value]: m_listsStates)
    {
        ROS_INFO("cherche right state %s in %i", state->type().name(), key);
        if(value.find(state->type()) != value.end())
        {
            value[state->type()]->disable();
            value[stateType]->enable(parameter, stateType);
        }
    } 
}

bool StateManager::isIdle(int indexMotivation){
    std::type_index idleType(typeid(IdleState));
    if(m_listsStates.at(indexMotivation).find(idleType) != m_listsStates.at(indexMotivation).end())
    {
        //IdleState exist in motivation indexMotivation
        if(m_listsStates[indexMotivation][idleType].get()->enabled()){
            return true;
        }
    }
    return false;
}
