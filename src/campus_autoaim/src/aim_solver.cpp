#include "aim_solver.hpp"

#include <cmath>

cv::Vec3d cameraToReference(
    const cv::Vec3d& positionCamera,
    double yawDegrees,
    double pitchDegrees
)
{
    const double x = positionCamera[0];
    const double y = positionCamera[1];
    const double z = positionCamera[2];

    const double yaw = yawDegrees * CV_PI / 180.0;

    const double pitch = pitchDegrees * CV_PI / 180.0;

    const double q = z * std::cos(pitch) - y * std::sin(pitch);

    const double referenceY = -y * std::cos(pitch) - z * std::sin(pitch);

    const double referenceX = x * std::cos(yaw) + q * std::sin(yaw);

    const double referenceZ = -x * std::sin(yaw) + q * std::cos(yaw);

    return cv::Vec3d(referenceX, referenceY, referenceZ);
}

cv::Vec3d referenceToCamera(
    const cv::Vec3d& positionReference,
    double yawDegrees,
    double pitchDegrees
)
{
    const double referenceX = positionReference[0];
    const double referenceY = positionReference[1];
    const double referenceZ = positionReference[2];

    const double yaw = yawDegrees * CV_PI / 180.0;
    const double pitch = pitchDegrees * CV_PI / 180.0;

    const double x = referenceX * std::cos(yaw) - referenceZ * std::sin(yaw);
    const double q = referenceX * std::sin(yaw) + referenceZ * std::cos(yaw);
    const double y = -referenceY * std::cos(pitch) - q * std::sin(pitch);
    const double z = -referenceY * std::sin(pitch) + q * std::cos(pitch);

    return cv::Vec3d(x, y, z);
}

cv::Vec3d cameraWorldPositionFromMap(
    const cv::Vec3d& headPositionMap,
    double yawDegrees,
    double pitchDegrees
)
{
    constexpr double sceneToMillimeters = 1000.0 / 3.0;
    
    constexpr double cameraForwardOffset = 80.0;

    const cv::Vec3d headPositionWorld(
        -headPositionMap[1],
        headPositionMap[2],
        headPositionMap[0]
    );

    return headPositionWorld * sceneToMillimeters
        + cameraToReference(
            cv::Vec3d(0.0, 0.0, cameraForwardOffset),
            yawDegrees,
            pitchDegrees
        );
}

AimResult solveAim(
    const cv::Vec3d& positionCamera,
    double currentYawDegrees,
    double currentPitchDegrees,
    double bulletSpeed
)
{
    AimResult result;

    const double x = positionCamera[0];
    const double y = positionCamera[1];
    const double z = positionCamera[2];

    if(!std::isfinite(x) ||
       !std::isfinite(y) ||
       !std::isfinite(z) ||
       !std::isfinite(currentYawDegrees) ||
       !std::isfinite(currentPitchDegrees) ||
       z <= 0.0)
    {
        return result;
    }

    cv::Vec3d positionReference =
        cameraToReference(
            positionCamera,
            currentYawDegrees,
            currentPitchDegrees
        );

    const double worldX = positionReference[0];
    const double worldY = positionReference[1];
    const double worldZ = positionReference[2];

    const double horizontalDistance = std::hypot(worldX,worldZ);

    if(horizontalDistance <= 0.0)
    {
        return result;
    }

    constexpr double gravity = 9810.0;

    const double v2 = bulletSpeed * bulletSpeed;

    const double discriminant =
        v2 * v2
        - gravity * (
            gravity
            * horizontalDistance
            * horizontalDistance
            + 2.0
            * worldY
            * v2
        );

    if(discriminant < 0.0)
    {
        return result;
    }

    const double launchAngle = std::atan((v2 - std::sqrt(discriminant))/(gravity * horizontalDistance));

    result.yaw =std::atan2(worldX,worldZ) * 180.0 / CV_PI;

    result.pitch = -launchAngle * 180.0 / CV_PI;

    if(!std::isfinite(result.yaw) ||
       !std::isfinite(result.pitch) ||
       result.pitch < -60.0 ||
       result.pitch > 60.0)
    {
        return AimResult{};
    }

    result.valid = true;

    return result;
}
