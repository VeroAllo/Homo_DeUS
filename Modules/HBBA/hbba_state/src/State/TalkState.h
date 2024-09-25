#ifndef HD_TALK_STATE_H
#define HD_TALK_STATE_H

#include "State.h"

class TalkState : public State, public DesireSetObserver
{
    uint64_t m_talkDesireId;

public:
    TalkState(
        StateManager& stateManager,
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::type_index nextStateType,
        std::int Intensite);
    ~TalkState() override;

    DECLARE_NOT_COPYABLE(TalkState);
    DECLARE_NOT_MOVABLE(TalkState);

    void onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& _) override;

protected:
    void enable(const std::string& parameter, const std::type_index& previousStageType) override;
    void disable() override;
    

private:
    virtual std::string generateText(const std::string& parameter) { return "";};
};

#endif

