#pragma once

#include <hbba_lite/core/Desire.h>
#include <hbba_lite/core/DesireSet.h>

#include <string>
#include <limits>

class GotoDesire : public Desire
{
public:
    explicit GotoDesire(uint16_t intensity = 1);
    ~GotoDesire() override = default;

    DECLARE_DESIRE_METHODS(GotoDesire); 
};

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

class SpeakDesire : public Desire
{
public:
    explicit SpeakDesire(uint16_t intensity = 1);
    ~SpeakDesire() override = default;

    DECLARE_DESIRE_METHODS(SpeakDesire); 
};
