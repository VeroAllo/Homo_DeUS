#include "StateManager.h"

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
    std::type_index previousStageType(typeid(State));
    std::type_index nextStageType(typeid(State));

    nextStageType = stateType;
    State* tmp_state;

    for(auto& [key, value]: m_listsStates)
    {
        ROS_INFO("cherche right state %s in %i", state->type().name(), key);
        if(value.find(state->type()) != value.end())
        {
            tmp_state = value[state->type()].get();
            if (state->type() != stateType)
            {
                ROS_INFO("Disabling %s", state->type().name());
                value[state->type()].get()->disable();
                previousStageType = state->type();
                nextStageType = state->nextStateType();
            }
            
            if( nextStageType == stateType)
            {
                value[stateType].get()->enable(parameter, stateType);
                ROS_INFO("Enabling %s (%s)", stateType.name(), parameter.c_str());
            }
        }
    }
    
}
