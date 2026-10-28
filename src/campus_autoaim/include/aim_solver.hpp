#ifndef AIM_SOLVER_HPP
#define AIM_SOLVER_HPP

#include <opencv2/core.hpp>

struct AimResult
{
    double yaw = 0.0;
    double pitch = 0.0;
    bool valid = false;
};

cv::Vec3d cameraToReference(
    const cv::Vec3d& positionCamera,
    double yawDegrees,
    double pitchDegrees
);

cv::Vec3d referenceToCamera(
    const cv::Vec3d& positionReference,
    double yawDegrees,
    double pitchDegrees
);

cv::Vec3d cameraWorldPositionFromMap(
    const cv::Vec3d& headPositionMap,
    double yawDegrees,
    double pitchDegrees
);

AimResult solveAim(
    const cv::Vec3d& positionCamera,
    double currentYawDegrees,
    double currentPitchDegrees,
    double bulletSpeed
);

#endif
