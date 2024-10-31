#include "DiscussionState.h"
#include "StateManager.h"
#include <homodeus_hbba_lite/HDDesires.h>

using namespace std;

DiscussionState::DiscussionState(
    StateManager& stateManager,
    std::shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    std::type_index nextStateType,
    int Intensite, std::string discussionContext)
    : State(stateManager, desireSet, nodeHandle, nextStateType, Intensite),
      m_discussDesireId(MAX_DESIRE_ID),
      m_discussionContext(discussionContext)
{
    ROS_INFO("state DiscussionState creer");
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
    ROS_INFO("Discussion started");
    State::enable(parameter, previousStageType);

    auto discussDesire = make_unique<DiscussDesire>(generateDiscussion(), m_Intensite);
    m_discussDesireId = discussDesire->id();
    // discussDesire.get()->setIntensity(m_Intensite);

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
    return m_discussionContext;
}
