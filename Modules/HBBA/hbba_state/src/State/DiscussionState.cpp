#include "DiscussionState.h"
#include "StateManager.h"
#include <homodeus_hbba_lite/HDDesires.h>

using namespace std;

DiscussionState::DiscussionState(
    StateManager& stateManager,
    std::shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType)
    : State(stateManager, desireSet, nodeHandle, nextStateType)
{
    m_desireSet->addObserver(this);
}

DiscussionState::~DiscussionState()
{
    m_desireSet->removeObserver(this);
}

void DiscussionState::onDesireSetChanged(const vector<unique_ptr<Desire>>& _)
{
    if (!enabled() || m_desireSet->contains(m_discussDesireId))
    {
        return;
    }

    m_stateManager.switchTo(this, m_nextStateType);
}

void DiscussionState::enable(const string& parameter, const type_index& previousStageType)
{
    State::enable(parameter, previousStageType);

    auto discussDesire = make_unique<DiscussDesire>(/* generateDiscussion() */);
    m_discussDesireId = discussDesire->id();

    m_desireIds.emplace_back(discussDesire->id());

    auto transaction = m_desireSet->beginTransaction();
    m_desireSet->addDesire(move(discussDesire));
}

void DiscussionState::disable()
{
    State::disable();
    m_discussDesireId = MAX_DESIRE_ID;
}

string DiscussionState::generateDiscussion()
{
    return "command1";
}
