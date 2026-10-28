#pragma once

#include <algorithm>
#include <array> 
#include <cmath>
#include <cstdint> 

#include "armor.hpp"

enum class SelectionState
{
    NONE,

    FIRST_SELECT,

    CONTINUE,

    SWITCH,

    REACQUIRE
};

struct SelectionResult
{
    int index = -1;

    SelectionState state = SelectionState::NONE;
};

struct BuildingAttackPermissions
{
    bool allowOutpost = false;
    bool allowBase = false;
};

class BuildingAttackRules
{
private:
    bool m_seenRobot4Alive = false;
    bool m_seenRobot5Alive = false;
    bool m_seenOutpostAlive = false;
    bool m_hasState = false;
    double m_lastTime = 0.0;
    double m_lastRemainingSeconds = 0.0;

public:
    BuildingAttackPermissions update(
        const std::array<std::int32_t, 7>& enemyHealths,
        double currentTime,
        double remainingSeconds
    );
};

class TargetSelector
{
private:
    bool m_locked = false;

    int m_lastNumber = -1;
    cv::Point2f m_lastCenter{0.0f, 0.0f};

    int m_lostCount = 0;
    int m_maxLostFrames = 3;
    double m_trackingRadius = 150.0;

    double scoreDistance(double distance) const;
    double scoreFacing(const PoseResult& pose) const;
    double scoreCenterContinuity(const cv::Point2f& center) const;
    double scoreNumberContinuity(int number) const;
    bool isPoseUsable(const PoseResult& pose) const;
    bool isAttackAllowed(int number, const BuildingAttackPermissions& permissions) const;
    double m_switchMargin = 20.0;
    double m_minAssociationScore = 30.0;

public:
    TargetSelector() = default;

    SelectionResult select(
        const std::vector<std::unique_ptr<Armor>>& armors,
        TargetColor targetColor,
        const BuildingAttackPermissions& permissions = {}
    );

    bool isLocked() const;
};
