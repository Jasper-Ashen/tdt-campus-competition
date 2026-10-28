#ifndef GIMBAL_CONTROLLER_HPP
#define GIMBAL_CONTROLLER_HPP

struct GimbalCommand
{
    double yaw = 0.0;
    double pitch = 0.0;
    bool valid = false;
    bool shoot = false;
};

struct MovementDirection
{
    double yaw = 0.0;
    bool valid = false;
};

class GimbalController
{
public:
    GimbalCommand update(
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
        bool outpostAimed = false
    );

private:
    MovementDirection movementDirection(
        double mapVx,
        double mapVy
    ) const;

    double normalizeAngle(double angle) const;

    double m_searchCenterYaw = 0.0;
    bool m_hasSearchCenter = false;

    double m_lostTargetStartTime = 0.0;
    bool m_hasLostTargetStartTime = false;

    static constexpr double pitchReturnDelay = 2.0;

    double m_targetSearchCenterYaw = 0.0;

    double m_lastUpdateTime = 0.0;
    bool m_hasLastUpdateTime = false;
    static constexpr double maxCenterYawSpeed = 90.0;

    static constexpr double yawShootTolerance = 1.0;
    static constexpr double pitchShootTolerance = 1.0;
    static constexpr double maxShootDistance = 7000.0;

};

#endif
