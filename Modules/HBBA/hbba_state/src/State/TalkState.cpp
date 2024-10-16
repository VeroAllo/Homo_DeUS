#include "TalkState.h"

#include "StateManager.h"

#include <homodeus_hbba_lite/HDDesires.h>

using namespace std;

TalkState::TalkState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    type_index nextStateType)
    : State(stateManager, desireSet, nodeHandle, nextStateType),
    m_talkDesireId(MAX_DESIRE_ID)
{
    m_desireSet->addObserver(this);
}

TalkState::~TalkState()
{
    m_desireSet->removeObserver(this);
}

void TalkState::onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& _)
{
    if (!enabled() || m_desireSet->contains(m_talkDesireId))
    {
        return;
    }

    m_stateManager.switchTo(this, m_nextStateType);
}

void TalkState::enable(const string& parameter, const type_index& previousStageType)
{
    State::enable(parameter, previousStageType);

    auto talkDesire = make_unique<TalkDesire>(generateText(parameter));
    m_talkDesireId = talkDesire->id();

    m_desireIds.emplace_back(talkDesire->id());

    auto transaction = m_desireSet->beginTransaction();
    m_desireSet->addDesire(move(talkDesire));
}

void TalkState::disable()
{
    State::disable();
    m_talkDesireId = MAX_DESIRE_ID;
}
