#ifndef HD_STATES_STATE_MANAGER_H
#define HD_STATES_STATE_MANAGER_H

#include "State.h"
#include <ros/ros.h>

#include <hbba_lite/utils/ClassMacros.h>

#include <memory>
#include <unordered_map>

class StateManager
{

public:
    std::unordered_map<int, std::unordered_map<std::type_index, std::unique_ptr<State>>> m_listsStates;
    StateManager();
    virtual ~StateManager();

    DECLARE_NOT_COPYABLE(StateManager);
    DECLARE_NOT_MOVABLE(StateManager);

    void addState(int indexList, std::unique_ptr<State> state);
    void addListStates(int indexList, std::unique_ptr<State> fistState);

    void switchTo(State* state, std::type_index type, const std::string& parameter = "");
    template<class T>
    void switchTo(int indexMotivation, const std::string& parameter = "");

    bool isIdle(int indexMotivation);

};

template<class T>
void StateManager::switchTo(int indexMotivation, const std::string& parameter)
{
    switchTo(m_listsStates[indexMotivation][std::type_index(typeid(T))].get(), std::type_index(typeid(T)), parameter);
}

#endif