#include "DropState.h"
#include "StateManager.h"

#include <homodeus_hbba_lite/HDDesires.h>

using namespace std;

DropState::DropState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    type_index nextStateType,
    std::int intensite)
    : State(stateManager, desireSet, nodeHandle, nextStateType, intensite),
      m_dropDesireId(MAX_DESIRE_ID)
{
    m_desireSet->addObserver(this);
}

DropState::~DropState()
{
    m_desireSet->removeObserver(this);
}

void DropState::onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& _)
{
    if (!enabled() || m_desireSet->contains(m_dropDesireId))
    {
        return;
    }

    m_stateManager.switchTo(this, m_nextStateType);
}

void DropState::enable(const string& parameter, const type_index& previousStageType)
{
    State::enable(parameter, previousStageType);

    auto dropDesire = make_unique<DropDesire>();
    m_dropDesireId = dropDesire->id();

    m_desireIds.emplace_back(dropDesire->id());

    auto transaction = m_desireSet->beginTransaction();
    m_desireSet->addDesire(move(dropDesire));
}

void DropState::disable()
{
    State::disable();
    m_dropDesireId = MAX_DESIRE_ID;
}