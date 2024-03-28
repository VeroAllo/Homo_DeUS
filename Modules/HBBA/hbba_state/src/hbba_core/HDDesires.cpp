#include "HDDesires.h"


GotoDesire::GotoDesire(std::string goal,uint16_t intensity) : Desire(intensity), m_goal(goal) {}

ExploreDesire::ExploreDesire(uint16_t intensity) : Desire(intensity) {}

TakeDesire::TakeDesire(uint16_t intensity) : Desire(intensity) {}

DropDesire::DropDesire(uint16_t intensity) : Desire(intensity) {}

FindDesire::FindDesire(uint16_t intensity) : Desire(intensity) {}

ListenDesire::ListenDesire(uint16_t intensity) : Desire(intensity) {}

SpeakDesire::SpeakDesire(std::string text, uint16_t intensity) : Desire(intensity), m_text(text) {}

DiscussDesire::DiscussDesire(uint16_t intensity) : Desire(intensity) {}