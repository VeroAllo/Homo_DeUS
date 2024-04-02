#ifndef HD_STATES_STATE_MANAGER_H
#define HD_STATES_STATE_MANAGER_H

#include "State.h"

#include <ros/ros.h>

#include <hbba_lite/utils/ClassMacros.h>

#include <memory>
#include <unordered_map>
#include <vector>


class StateManager
{
    std::unordered_map<int, std::unordered_map<std::type_index, std::unique_ptr<State>>> m_listStates{};

public:
    StateManager();
    virtual ~StateManager();

    DECLARE_NOT_COPYABLE(StateManager);
    DECLARE_NOT_MOVABLE(StateManager);

    void addState(std::unique_ptr<State> state, int listId);

    State* findState(State* currentState, std::type_index stateType);
    void startStateMachine(int listKey);
    // void addListState(std::unique_ptr<State> firstState, int listId);

    template<class T>
    void switchTo(State* currentState, const std::string& parameter = "");
    void switchTo(State* currentState, std::type_index type, const std::string& parameter = "");
    
};

template<class T>
void StateManager::switchTo(State* currentState,  const std::string& parameter)
{
    switchTo(currentState, std::type_index(typeid(T)), parameter);
}

#endif