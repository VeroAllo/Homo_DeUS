#pragma once

#include <hbba_lite/core/Desire.h>
#include <hbba_lite/core/DesireSet.h>
#include <std_msgs/String.h>

#include <string>
#include <limits>

class Desire;

class GotoDesire : public Desire
{
    std::string m_goal;
public:
    explicit GotoDesire(std::string goal, uint16_t intensity = 1);
    ~GotoDesire() = default;

    uint64_t id() const { return Desire::id(); }

    DECLARE_DESIRE_METHODS(GotoDesire);
    const std::string& goal() const;
};

inline const std::string& GotoDesire::goal() const
{
    return m_goal;
}

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

class SpeakDesire : public Desire
{
    std::string m_text;
public:
    explicit SpeakDesire(std::string text, uint16_t intensity = 1);
    ~SpeakDesire() = default;

    uint64_t id() const { return Desire::id(); }

    DECLARE_DESIRE_METHODS(SpeakDesire); 
    const std::string& text() const;
};

inline const std::string& SpeakDesire::text() const
{
    return m_text;
}


class DiscussDesire : public Desire
{
public:
    explicit DiscussDesire(uint16_t intensity = 1);
    ~DiscussDesire() = default;

    uint64_t id() const { return Desire::id(); }

    DECLARE_DESIRE_METHODS(DiscussDesire); 
};
