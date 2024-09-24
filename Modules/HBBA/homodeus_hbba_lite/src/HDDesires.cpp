#include <homodeus_hbba_lite/HDDesires.h>

GotoDesire::GotoDesire(std::string destinationInText, uint16_t intensity) : Desire(intensity), m_DestinationInText(destinationInText) {}

ExploreDesire::ExploreDesire(uint16_t intensity) : Desire(intensity) {}

TakeDesire::TakeDesire(std::string commande, uint16_t intensity) : Desire(intensity), m_Commande(commande) {}

DropDesire::DropDesire(uint16_t intensity) : Desire(intensity) {}

FindDesire::FindDesire(uint16_t intensity) : Desire(intensity) {}

ListenDesire::ListenDesire(uint16_t intensity) : Desire(intensity) {}

TalkDesire::TalkDesire(std::string textToTalk, uint16_t intensity) : Desire(intensity), m_TextToTalk(textToTalk) {}

DiscussDesire::DiscussDesire(uint16_t intensity) : Desire(intensity) {}
