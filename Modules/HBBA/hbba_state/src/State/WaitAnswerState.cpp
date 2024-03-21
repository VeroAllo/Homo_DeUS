#include "WaitAnswerState.h"
#include "InvalidTaskState.h"

#include "StateManager.h"

//#include <t_top_hbba_lite/Desires.h>

using namespace std;

static const string ENGLISH_WEATHER_WORD = "weather";
static const string ENGLISH_FORECAST_WORD = "forecast";
static const string ENGLISH_STORY_WORD = "story";
static const string ENGLISH_DANCE_WORD = "dance";
static const string ENGLISH_SONG_WORD = "song";


WaitAnswerState::WaitAnswerState(
    StateManager& stateManager,
    shared_ptr<DesireSet> desireSet,
    ros::NodeHandle& nodeHandle)
    : State(stateManager, desireSet, nodeHandle),
      m_transcriptReceived(false)
{
    m_speechToTextSubscriber =
        nodeHandle.subscribe("speech_to_text/transcript", 1, &WaitAnswerState::speechToTextSubscriberCallback, this);
}

void WaitAnswerState::enable(const string& parameter, const type_index& previousStageType)
{
    State::enable(parameter, previousStageType);
    m_transcriptReceived = false;

    auto speechToTextDesire = make_unique<SpeechToTextDesire>();

    m_desireIds.emplace_back(speechToTextDesire->id());

    auto transaction = m_desireSet->beginTransaction();
    m_desireSet->addDesire(move(speechToTextDesire));

    constexpr bool ONE_SHOT = true;
    m_timeoutTimer =
        m_nodeHandle.createTimer(ros::Duration(TIMEOUT_S), &WaitAnswerState::timeoutTimerCallback, this, ONE_SHOT);
}

void WaitAnswerState::disable()
{
    State::disable();

    if (m_timeoutTimer.isValid())
    {
        m_timeoutTimer.stop();
    }
}

void WaitAnswerState::speechToTextSubscriberCallback(const speech_to_text::Transcript::ConstPtr& msg)
{
    if (!enabled())
    {
        return;
    }

    m_transcriptReceived = true;
    switchStateAfterTranscriptReceived(msg->text, msg->is_final);
}

void WaitAnswerState::timeoutTimerCallback(const ros::TimerEvent& event)
{
    if (!enabled())
    {
        return;
    }

    switchStateAfterTimeout(m_transcriptReceived);
}
