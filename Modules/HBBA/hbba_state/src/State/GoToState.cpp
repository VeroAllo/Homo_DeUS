#include "GoToState.h"
#include "StateManager.h"
#include <homodeus_hbba_lite/HDDesires.h>

using namespace std;

GoToState::GoToState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    type_index nextStateType)
    : State(stateManager, desireSet, nodeHandle, nextStateType),
      m_gotoDesireId(MAX_DESIRE_ID)
{
    m_desireSet->addObserver(this);
}

GoToState::~GoToState()
{
    m_desireSet->removeObserver(this);
}

void GoToState::onDesireSetChanged(const vector<unique_ptr<Desire>>& _)
{
    ROS_INFO("Goto desireSet changed");
    if(!enabled() || m_desireSet->contains(m_gotoDesireId))
    {
        ROS_INFO("We got inside");
        return;
    }

    m_stateManager.switchTo(this, m_nextStateType);
}

void GoToState::enable(const string& parameter, const type_index& previousStageType)
{
    State::enable(parameter, previousStageType);
    auto gotoDesire = make_unique<GotoDesire>(generateGoal(parameter));
    
    m_gotoDesireId = gotoDesire->id();

    m_desireIds.emplace_back(gotoDesire->id());

    auto transaction = m_desireSet->beginTransaction();
    m_desireSet->addDesire(move(gotoDesire));
}

void GoToState::disable()
{
    State::disable();
    m_gotoDesireId = MAX_DESIRE_ID;
}