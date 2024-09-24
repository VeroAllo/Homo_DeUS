#pragma once

#include <hbba_lite/core/Desire.h>
#include <hbba_lite/core/DesireSet.h>
#include <std_msgs/String.h>

#include <string>
#include <limits>

class GotoDesire : public Desire
{
public:
    explicit GotoDesire(std::string destinationInText, uint16_t intensity = 1);
    ~GotoDesire() = default;

    uint64_t id() const { return Desire::id(); }

    DECLARE_DESIRE_METHODS(GotoDesire); 
    std::string m_DestinationInText{};
};

class ExploreDesire : public Desire
{
public:
    explicit ExploreDesire(uint16_t intensity = 1);
    ~ExploreDesire() = default;

    uint64_t id() const { return Desire::id(); }

    DECLARE_DESIRE_METHODS(ExploreDesire); 
};

class TakeDesire : public Desire
{
public:
    explicit TakeDesire(uint16_t intensity = 1);
    ~TakeDesire() = default;

    uint64_t id() const { return Desire::id(); }

    DECLARE_DESIRE_METHODS(TakeDesire); 
};

class DropDesire : public Desire
{
public:
    explicit DropDesire(uint16_t intensity = 1);
    ~DropDesire() = default;

    uint64_t id() const { return Desire::id(); }

    DECLARE_DESIRE_METHODS(DropDesire); 
};

class FindDesire : public Desire
{
public:
    explicit FindDesire(uint16_t intensity = 1);
    ~FindDesire() = default;

    uint64_t id() const { return Desire::id(); }

    DECLARE_DESIRE_METHODS(FindDesire); 
};

class ListenDesire : public Desire
{
public:
    explicit ListenDesire(uint16_t intensity = 1);
    ~ListenDesire() = default;

    uint64_t id() const { return Desire::id(); }

    DECLARE_DESIRE_METHODS(ListenDesire); 
};

class TalkDesire : public Desire
{
public:
    explicit TalkDesire(std::string textToTalk, uint16_t intensity = 1);
    ~TalkDesire() = default;

    uint64_t id() const { return Desire::id(); }
    std::string getMessage() const { return m_TextToTalk;}

    DECLARE_DESIRE_METHODS(TalkDesire); 

private:
    std::string m_TextToTalk{};
};

class DiscussDesire : public Desire
{
public:
    explicit DiscussDesire(uint16_t intensity = 1);
    ~DiscussDesire() = default;

    uint64_t id() const { return Desire::id(); }

    DECLARE_DESIRE_METHODS(DiscussDesire); 
};
