#include "DropState.h"
#include "StateManager.h"


#include "../hbba_core/HDDesires.h"

using namespace std;

DropState::DropState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    type_index nextStateType)
    : State(stateManager, desireSet, nodeHandle),
      m_nextStateType(nextStateType),
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

    m_stateManager.switchTo(m_nextStateType);
}

void DropState::enable(const string& parameter, const type_index& previousStageType)
{
    State::enable(parameter, previousStageType);

    auto talkDesire = make_unique<TalkDesire>(generateDrop());
    m_dropDesireId = talkDesire->id();

    m_desireIds.emplace_back(talkDesire->id());

    auto transaction = m_desireSet->beginTransaction();
    m_desireSet->addDesire(move(talkDesire));
}

void DropState::disable()
{
    State::disable();
    m_dropDesireId = MAX_DESIRE_ID;
}

string DropState::generateDrop()
{
    return "drop";
}