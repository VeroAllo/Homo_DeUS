#include "InvalidTaskState.h"

#include "StateManager.h"

//#include <t_top_hbba_lite/Desires.h>

using namespace std;

InvalidTaskState::InvalidTaskState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    type_index nextStateType)
    : State(stateManager, desireSet, nodeHandle),
      m_nextStateType(nextStateType),
      m_talkDesireId(MAX_DESIRE_ID),
      m_gestureDesireId(MAX_DESIRE_ID)
{
    m_desireSet->addObserver(this);
}

InvalidTaskState::~InvalidTaskState()
{
    m_desireSet->removeObserver(this);
}

void InvalidTaskState::onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& _)
{
    if (!enabled() || m_desireSet->contains(m_talkDesireId))
    {
        return;
    }

    m_stateManager.switchTo(m_nextStateType);
}

void InvalidTaskState::enable(const string& parameter, const type_index& previousStageType)
{
    State::enable(parameter, previousStageType);

    auto talkDesire = make_unique<TalkDesire>(generateText());
    m_talkDesireId = talkDesire->id();

    m_desireIds.emplace_back(talkDesire->id());

    auto transaction = m_desireSet->beginTransaction();
    m_desireSet->addDesire(move(talkDesire));
}

void InvalidTaskState::disable()
{
    State::disable();

    m_talkDesireId = MAX_DESIRE_ID;
    m_gestureDesireId = MAX_DESIRE_ID;
}

string InvalidTaskState::generateText()
{
    return "I cannot do that.";

}
