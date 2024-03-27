#pragma once

#include <hbba_lite/core/Desire.h>
#include <hbba_lite/core/DesireSet.h>

#include <string>
#include <limits>


class SpeechToTextDesire : public Desire
{
public:
    explicit SpeechToTextDesire(uint16_t intensity = 1);
    ~SpeechToTextDesire() override = default;

    DECLARE_DESIRE_METHODS(SpeechToTextDesire)
};


class TalkDesire : public Desire
{
    std::string m_text;

public:
    explicit TalkDesire(std::string text, uint16_t intensity = 1);
    ~TalkDesire() override = default;

    DECLARE_DESIRE_METHODS(TalkDesire)

    const std::string& text() const;
};

inline const std::string& TalkDesire::text() const
{
    return m_text;
}

class GotoDesire : public Desire
{
    std::string m_goal;

public:
    explicit GotoDesire(std::string goal, uint16_t intensity = 1);
    ~GotoDesire() override = default;

    DECLARE_DESIRE_METHODS(GotoDesire)

    const std::string& goal() const;
};

inline const std::string& GotoDesire::goal() const
{
    return m_goal;
}

class DiscussDesire : public Desire
{
    std::string m_text;

public:
    explicit DiscussDesire(std::string goal, uint16_t intensity = 1);
    ~DiscussDesire() override = default;

    DECLARE_DESIRE_METHODS(DiscussDesire)

    const std::string& text() const;
};

inline const std::string& DiscussDesire::text() const
{
    return m_text;
}

class ExploreDesire : public Desire
{
public:
    explicit ExploreDesire(uint16_t intensity = 1);
    ~ExploreDesire() override = default;

    DECLARE_DESIRE_METHODS(ExploreDesire); 
};

class TakeDesire : public Desire
{
public:
    explicit TakeDesire(uint16_t intensity = 1);
    ~TakeDesire() override = default;

    DECLARE_DESIRE_METHODS(TakeDesire); 
};

class DropDesire : public Desire
{
public:
    explicit DropDesire(uint16_t intensity = 1);
    ~DropDesire() override = default;

    DECLARE_DESIRE_METHODS(DropDesire); 
};

class FindDesire : public Desire
{
public:
    explicit FindDesire(uint16_t intensity = 1);
    ~FindDesire() override = default;

    DECLARE_DESIRE_METHODS(FindDesire); 
};

class ListenDesire : public Desire
{
public:
    explicit ListenDesire(uint16_t intensity = 1);
    ~ListenDesire() override = default;

    DECLARE_DESIRE_METHODS(ListenDesire); 
};
