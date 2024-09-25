#include "State.h"

using namespace std;

State::State(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle, 
    std::type_index nextStateType,
    std::int Intensite)
    : m_enabled(false),
      m_stateManager(stateManager),
      m_desireSet(move(desireSet)),
      m_nodeHandle(nodeHandle),
      m_nextStateType(nextStateType),
      m_previousStageType(typeid(State)),
      m_Intensite(Intensite)

{
}
State::~State(){}

void State::enable(const string& parameter, const type_index& previousStageType)
{
    m_enabled = true;
    m_previousStageType = previousStageType;
}

void State::disable()
{
    m_enabled = false;
}