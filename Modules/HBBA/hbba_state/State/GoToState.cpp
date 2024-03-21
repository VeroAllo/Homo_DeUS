#include "GoToState.h"
#include "StateManager.h"

//#include <t_top_hbba_lite/Desires.h>

using namespace std;

GoToState::GoToState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    type_index nextStateType)
    : State(stateManager, desireSet, nodeHandle),
      m_nextStateType(nextStateType),
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
    if(!enabled() || m_desireSet->contains(m_gotoDesireId))
    {
        return;
    }

    m_stateManager.switchTo(m_nextStateType);
}

void GoToState::enable(const string& parameter, const type_index& previousStageType)
{
    State::enable(parameter, previousStageType);
    auto gotoDesire = make_unique<GoToDesire>(generateGoal(parameter));
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