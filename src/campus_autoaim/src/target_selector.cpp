#include "target_selector.hpp"

BuildingAttackPermissions BuildingAttackRules::update(
    const std::array<std::int32_t, 7>& enemyHealths,
    double currentTime,
    double remainingSeconds
)
{
    BuildingAttackPermissions permissions;

    if(!std::isfinite(currentTime) || currentTime <= 0.0 ||
       !std::isfinite(remainingSeconds) || remainingSeconds <= 0.0)
    {
        *this = BuildingAttackRules{};
        return permissions;
    }

    if(m_hasState &&
       (currentTime < m_lastTime || remainingSeconds > m_lastRemainingSeconds))
    {
        *this = BuildingAttackRules{};
    }
    m_hasState = true;
    m_lastTime = currentTime;
    m_lastRemainingSeconds = remainingSeconds;

    const int robot4Health = enemyHealths[3];
    const int robot5Health = enemyHealths[4];
    const int outpostHealth = enemyHealths[5];
    const int baseHealth = enemyHealths[6];

    if(std::any_of(enemyHealths.begin(), enemyHealths.end(), [](std::int32_t health)
       {
           return health < 0;
       }) ||
       std::all_of(enemyHealths.begin(), enemyHealths.end(), [](std::int32_t health)
       {
           return health == 0;
       }))
    {
        return permissions;
    }

    m_seenRobot4Alive = m_seenRobot4Alive || robot4Health > 0;
    m_seenRobot5Alive = m_seenRobot5Alive || robot5Health > 0;
    m_seenOutpostAlive = m_seenOutpostAlive || outpostHealth > 0;

    permissions.allowOutpost =
        m_seenRobot4Alive && m_seenRobot5Alive &&
        robot4Health == 0 && robot5Health == 0 && outpostHealth > 0;
    permissions.allowBase =
        m_seenOutpostAlive && outpostHealth == 0 && baseHealth > 0;
    return permissions;
}

bool TargetSelector::isAttackAllowed(
    int number,
    const BuildingAttackPermissions& permissions
) const
{
    if(number == 6)
        return permissions.allowOutpost;
    if(number == 8)
        return permissions.allowBase;
    return true;
}



double TargetSelector::scoreDistance(double distance) const
{
    constexpr double nearDistance = 1000.0;
    constexpr double farDistance = 5000.0;

    double score =
        (farDistance - distance) /
        (farDistance - nearDistance)
        * 100.0;

    return std::clamp(score, 0.0, 100.0);
}

double TargetSelector::scoreFacing(
    const PoseResult& pose
) const
{
    cv::Mat rotationMatrix;
    cv::Rodrigues(
        pose.rvec,
        rotationMatrix
    );

    cv::Vec3d normal(
        rotationMatrix.at<double>(0, 2),
        rotationMatrix.at<double>(1, 2),
        rotationMatrix.at<double>(2, 2)
    );

    cv::Vec3d viewDirection =
        pose.tvec / pose.distance;

    double facingCos =
        std::abs(normal.dot(viewDirection));

    return std::clamp(
        facingCos * 100.0,
        0.0,
        100.0
    );
}

double TargetSelector::scoreCenterContinuity(
    const cv::Point2f& center
) const
{
    if(!m_locked)
        return 0.0;

    constexpr double maxPixelDistance = 150.0;

    double pixelDistance =
        cv::norm(center - m_lastCenter);

    double score =
        (1.0 - pixelDistance / maxPixelDistance)
        * 100.0;

    return std::clamp(
        score,
        0.0,
        100.0
    );
}

double TargetSelector::scoreNumberContinuity(
    int number
) const
{
    if(!m_locked)
        return 0.0;

    if(number == m_lastNumber)
        return 100.0;

    return 0.0;
}

bool TargetSelector::isPoseUsable(
    const PoseResult& pose
) const
{
    if(!pose.valid)
        return false;

    if(!std::isfinite(pose.distance) ||
       pose.distance <= 0.0)
    {
        return false;
    }

    for(int i = 0; i < 3; i++)
    {
        if(!std::isfinite(pose.rvec[i]) ||
           !std::isfinite(pose.tvec[i]))
        {
            return false;
        }
    }

    if(pose.tvec[2] <= 0.0)
        return false;

    return true;
}

SelectionResult TargetSelector::select(
    const std::vector<std::unique_ptr<Armor>>& armors,
    TargetColor targetColor,
    const BuildingAttackPermissions& permissions 
)
{
    const bool wasLocked = m_locked;
    if(m_locked && !isAttackAllowed(m_lastNumber, permissions))
    {
        m_locked = false;
        m_lostCount = 0;
        m_lastNumber = -1;
        m_lastCenter = cv::Point2f(0.0f, 0.0f);
    }

    if(m_locked)
    {
        int associatedIndex = -1;
        double bestAssociationScore = -1.0;

        for(size_t i = 0; i < armors.size(); i++)
        {
            const Armor& armor = *armors[i];

            if(armor.getColor() != targetColor)
                continue;

            if(!isAttackAllowed(armor.number().value(), permissions))
                continue;

            const PoseResult& pose = armor.pose();

            if(!isPoseUsable(pose))continue;

            double pixelDistance =
                cv::norm(
                    armor.getCenter() - m_lastCenter
                );

            if(pixelDistance > m_trackingRadius)
                continue;

            double centerScore =
                scoreCenterContinuity(
                    armor.getCenter()
                );

            double numberScore =
                scoreNumberContinuity(
                    armor.number().value()
                );

            double associationScore =
                centerScore
                + numberScore;

            if(associationScore > bestAssociationScore)
            {
                bestAssociationScore =
                    associationScore;

                associatedIndex =
                    static_cast<int>(i);
            }
        }

        if(associatedIndex >= 0 && bestAssociationScore >= m_minAssociationScore)
        {
            const Armor& associated =
                *armors[associatedIndex];

            const PoseResult& associatedPose =
                associated.pose();

            double associatedPriority;

            if(associated.number().value() == 6)
            {
                associatedPriority =
                    scoreFacing(associatedPose);
            }
            else
            {
                associatedPriority =
                    scoreDistance(associatedPose.distance)
                    +
                    scoreFacing(associatedPose);
            }

            int challengerIndex = -1;
            double bestChallengerPriority = -1.0;

            for(size_t i = 0; i < armors.size(); i++)
            {
                if(static_cast<int>(i) == associatedIndex)
                    continue;

                const Armor& armor =
                    *armors[i];

                if(armor.getColor() != targetColor)
                    continue;

                if(!isAttackAllowed(armor.number().value(), permissions))
                    continue;

                const PoseResult& pose =
                    armor.pose();

                if(!isPoseUsable(pose))continue;

                double priority;

                if(armor.number().value() == 6)
                {
                    priority =
                        scoreFacing(pose);
                }
                else
                {
                    priority =
                        scoreDistance(pose.distance)
                        +
                        scoreFacing(pose);
                }

                if(priority > bestChallengerPriority)
                {
                    bestChallengerPriority =
                        priority;

                    challengerIndex =
                        static_cast<int>(i);
                }
            }

            int selectedIndex = associatedIndex;

            if(challengerIndex >= 0 &&
               bestChallengerPriority >
                   associatedPriority + m_switchMargin)
            {
                selectedIndex = challengerIndex;
            }

            const Armor& selected =
                *armors[selectedIndex];

            m_lastCenter =
                selected.getCenter();

            m_lastNumber =
                selected.number().value();

            m_lostCount = 0;

            if(selectedIndex == associatedIndex)
            {
                return {selectedIndex, SelectionState::CONTINUE};
            }

            return {selectedIndex, SelectionState::SWITCH};
        }
    }

    int bestIndex = -1;
    double bestScore = -1.0;

    for(size_t i = 0; i < armors.size(); i++)
    {
        const Armor& armor = *armors[i];

        if(armor.getColor() != targetColor)
            continue;


        if(!isAttackAllowed(armor.number().value(), permissions))
            continue;

        const PoseResult& pose = armor.pose();

        if(!isPoseUsable(pose))continue;

        double facingScore =
            scoreFacing(pose);

        double score;

        if(armor.number().value() == 6)
        {
            score = facingScore;
        }
        else
        {
            double distanceScore = scoreDistance(pose.distance);

            score = distanceScore + facingScore;
        }

        if(score > bestScore)
        {
            bestScore = score;

            bestIndex = static_cast<int>(i);
        }
    }

    if(bestIndex >= 0)
    {
        const Armor& selected =
            *armors[bestIndex];

        m_lastCenter =
            selected.getCenter();

        m_lastNumber =
            selected.number().value();

        m_locked = true;
        m_lostCount = 0;

        if(wasLocked)
        {
            return {bestIndex, SelectionState::REACQUIRE};
        }

        return {bestIndex, SelectionState::FIRST_SELECT};
    }

    if(m_locked)
    {
        m_lostCount++;

        if(m_lostCount > m_maxLostFrames)
        {
            m_locked = false;
            m_lostCount = 0;
        }
    }

    return {-1, SelectionState::NONE};
}

bool TargetSelector::isLocked() const
{
    return m_locked;
}

