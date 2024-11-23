#include "TakeState.h"
#include "StateManager.h"
#include <homodeus_hbba_lite/HDDesires.h>

using namespace std;

TakeState::TakeState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle,
    type_index nextStateType,
    int intensite)
    : State(stateManager, desireSet, nodeHandle, nextStateType, intensite),
      m_takeDesireId(MAX_DESIRE_ID)
{
    m_desireSet->addObserver(this);
}

TakeState::~TakeState()
{
    m_desireSet->removeObserver(this);
}

void TakeState::onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& _)
{
    if (!enabled() || m_desireSet->contains(m_takeDesireId))
    {
        return;
    }

    m_stateManager.switchTo(this, m_nextStateType);
}

void TakeState::enable(const string& parameter, const type_index& previousStageType)
{
    State::enable(parameter, previousStageType);

    auto takeDesire = make_unique<TakeDesire>(GetCommandeObject());
    m_takeDesireId = takeDesire->id();
    m_desireIds.emplace_back(takeDesire->id());
    
    auto transaction = m_desireSet->beginTransaction();
    m_desireSet->addDesire(move(takeDesire));
}

void TakeState::disable()
{
    State::disable();
    m_Commande.clear();
    m_takeDesireId = MAX_DESIRE_ID;
}

void TakeState::GenerateObjectToTake(const std::string& parameter)
{
    m_Commande = parameter;
}