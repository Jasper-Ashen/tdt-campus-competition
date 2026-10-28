#include "gimbal_controller.hpp"

#include <cmath>
#include <opencv2/core.hpp>

MovementDirection GimbalController::movementDirection(
    double mapVx,
    double mapVy
) const
{
    double speed =
        std::hypot(mapVx, mapVy);

    constexpr double minMoveSpeed = 0.1;

    MovementDirection result;

    if(speed < minMoveSpeed)
    {
        result.valid=false;
        return result;
    }

    double unityVx = -mapVy;
    double unityVz = mapVx;

    double yaw =
        std::atan2(unityVx, unityVz)
        * 180.0 / CV_PI;

    result.yaw = yaw;
    result.valid = true;

    return result;
}

double GimbalController::normalizeAngle(
    double angle
) const
{
    while(angle > 180.0)
    {
        angle -= 360.0;
    }

    while(angle <= -180.0)
    {
        angle += 360.0;
    }

    return angle;
}

GimbalCommand GimbalController::update(
    bool locked,
    bool targetValid,
    double targetYaw,
    double targetPitch,
    double targetDistance,
    int targetNumber,
    double currentYaw,
    double currentPitch,
    double mapVx,
    double mapVy,
    double currentTime,
    bool outpostAimed // [OutpostPixelFire] 只影响6号的对准判断，不改变TRACK/HOLD/SEARCH。
)
{
    GimbalCommand command;

    if(locked)
    {
        m_hasLostTargetStartTime = false;
    }
    else if(!m_hasLostTargetStartTime)
    {
        m_lostTargetStartTime = currentTime;
        m_hasLostTargetStartTime = true;
    }

    bool returnPitchToHorizon =
        m_hasLostTargetStartTime &&
        currentTime - m_lostTargetStartTime >= pitchReturnDelay;

    double searchPitch =
        returnPitchToHorizon
        ? 0.0
        : currentPitch;

    if(targetValid)
    {
        command.yaw = targetYaw;
        command.pitch = targetPitch;
        command.valid = true;

        double yawError =normalizeAngle( targetYaw - currentYaw );

        double pitchError = targetPitch - currentPitch;

        // [OutpostPixelFire] 前哨用图像准星+板面角替换旧2度门槛；其他目标仍用原1度条件。
        bool aimed =
            targetNumber == 6
            ? outpostAimed
            : std::abs(yawError) <= yawShootTolerance &&
              std::abs(pitchError) <= pitchShootTolerance;

        bool distanceOk =
            targetDistance > 0.0 &&
            targetDistance <= maxShootDistance;

        command.shoot = aimed && distanceOk ;

        m_hasLastUpdateTime = false;

        return command;
    }

    if(locked)
    {
        command.yaw = currentYaw;
        command.pitch = currentPitch;
        command.valid = true;

        m_hasLastUpdateTime = false;

        return command;
    }

    MovementDirection direction =movementDirection(mapVx,mapVy);

    if(!m_hasSearchCenter || !m_hasLastUpdateTime)
    {
        m_targetSearchCenterYaw = currentYaw;
        m_searchCenterYaw = currentYaw;

        m_hasSearchCenter = true;
    }

    if(direction.valid)
    {
        double difference =normalizeAngle(direction.yaw - m_targetSearchCenterYaw);

        if(std::abs(difference) <= 120.0)
        {
            m_targetSearchCenterYaw = direction.yaw;
        }
        else
        {
            m_lastUpdateTime = currentTime;
            m_hasLastUpdateTime = true;
            command.yaw = m_searchCenterYaw;
            command.pitch = searchPitch;
            command.valid = true;
            return command;
        }
    }

    if(!m_hasLastUpdateTime)
    {
        m_lastUpdateTime = currentTime;

        m_hasLastUpdateTime = true;
    }
    else
    {
        double dt = currentTime - m_lastUpdateTime;

        m_lastUpdateTime = currentTime;

        if(dt > 0.0)
        {
            double centerDifference = normalizeAngle(m_targetSearchCenterYaw - m_searchCenterYaw);

            double maxStep =maxCenterYawSpeed * dt;

            if(std::abs(centerDifference) <= maxStep)
            {
                m_searchCenterYaw = m_targetSearchCenterYaw;
            }
            else if(centerDifference > 0.0)
            {
                m_searchCenterYaw += maxStep;
            }
            else
            {
                m_searchCenterYaw -= maxStep;
            }

            m_searchCenterYaw = normalizeAngle(m_searchCenterYaw);
        }
    }

    command.yaw = m_searchCenterYaw;

    command.pitch = searchPitch;

    command.valid = true;

    return command;
}
