#include "HDDesires.h"

using namespace std;


SpeechToTextDesire::SpeechToTextDesire(uint16_t intensity) : Desire(intensity) {}

TalkDesire::TalkDesire(string text, uint16_t intensity) : Desire(intensity), m_text(move(text)) {}

GotoDesire::GotoDesire(string goal, uint16_t intensity) : Desire(intensity), m_goal(move(goal)) {}

DiscussDesire::DiscussDesire(string text, uint16_t intensity) : Desire(intensity), m_text(move(text)) {}

ExploreDesire::ExploreDesire(uint16_t intensity) : Desire(intensity) {}

TakeDesire::TakeDesire(uint16_t intensity) : Desire(intensity) {}

DropDesire::DropDesire(uint16_t intensity) : Desire(intensity) {}

FindDesire::FindDesire(uint16_t intensity) : Desire(intensity) {}

ListenDesire::ListenDesire(uint16_t intensity) : Desire(intensity) {}