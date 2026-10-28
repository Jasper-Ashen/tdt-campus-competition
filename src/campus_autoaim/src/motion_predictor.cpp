#include "motion_predictor.hpp"
#include "aim_solver.hpp"

#include <cmath>
#include <opencv2/calib3d.hpp>

namespace
{
constexpr double maxResidualMm = 500.0;

bool isFinitePosition(const cv::Vec3d& position)
{
    return std::isfinite(position[0])
        && std::isfinite(position[1])
        && std::isfinite(position[2]);
}
}

void MotionPredictor::reset()
{
    m_lastWorldPosition = cv::Vec3d(0.0, 0.0, 0.0);
    m_velocityWorld = cv::Vec3d(0.0, 0.0, 0.0);
    m_lastTime = 0.0;
    m_hasLastWorldPosition = false;
    m_velocityValid = false;
    m_trustState = TrustState::EMPTY;
    m_hasProvisionalVelocity = false;
}

bool MotionPredictor::update(
    const cv::Vec3d& currentWorldPosition,
    double currentTime
)
{
    if(!m_hasLastWorldPosition)
    {
        m_lastWorldPosition = currentWorldPosition;
        m_lastTime = currentTime;
        m_hasLastWorldPosition = true;
        m_velocityValid = false;
        m_trustState = TrustState::WARMUP;
        m_hasProvisionalVelocity = false;
        return false;
    }

    constexpr double minDt = 0.02;
    constexpr double maxDt = 0.15;
    double dt = currentTime - m_lastTime;

    if(!std::isfinite(dt) || dt <= 0.0 || dt > maxDt)
    {
        reset();
        m_lastWorldPosition = currentWorldPosition;
        m_lastTime = currentTime;
        m_hasLastWorldPosition = true;
        m_trustState = TrustState::WARMUP;
        return false;
    }

    if(dt < minDt)
    {
        return false;
    }

    if(m_trustState == TrustState::WARMUP && !m_hasProvisionalVelocity)
    {
        cv::Vec3d provisionalVelocity =
            (currentWorldPosition - m_lastWorldPosition) / dt;
        if(!isFinitePosition(provisionalVelocity))return false;

        m_velocityWorld = provisionalVelocity;
        m_lastWorldPosition = currentWorldPosition;
        m_lastTime = currentTime;
        m_hasProvisionalVelocity = true;
        return false;
    }

    cv::Vec3d expectedPosition = m_lastWorldPosition + m_velocityWorld * dt;
    double residual = cv::norm(currentWorldPosition - expectedPosition);
    if(!isFinitePosition(expectedPosition) ||
       !std::isfinite(residual) || residual > maxResidualMm)
    {
        return false;
    }

    cv::Vec3d currentVelocity =
        (currentWorldPosition - m_lastWorldPosition) / dt;
    if(!isFinitePosition(currentVelocity))
        return false;

    m_velocityWorld = currentVelocity;
    m_lastWorldPosition = currentWorldPosition;
    m_lastTime = currentTime;
    m_velocityValid = true;
    m_trustState = TrustState::TRUSTED;
    m_hasProvisionalVelocity = false;
    return true;
}

cv::Vec3d MotionPredictor::predict3D(
    const cv::Vec3d& currentWorldPosition,
    double currentDistance,
    double bulletSpeed
) const
{
    double flightTime = currentDistance / bulletSpeed;
    return currentWorldPosition + m_velocityWorld * flightTime;
}

cv::Point2f MotionPredictor::projectToImage(
    const cv::Vec3d& position,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs
) const
{
    std::vector<cv::Point3d> objectPoints =
    {
        cv::Point3d(position[0], position[1], position[2])
    };
    std::vector<cv::Point2d> imagePoints;
    cv::Vec3d zeroRvec(0.0, 0.0, 0.0);
    cv::Vec3d zeroTvec(0.0, 0.0, 0.0);

    cv::projectPoints(
        objectPoints,
        zeroRvec,
        zeroTvec,
        cameraMatrix,
        distCoeffs,
        imagePoints
    );

    if(imagePoints.empty() ||
       !std::isfinite(imagePoints[0].x) ||
       !std::isfinite(imagePoints[0].y))
    {
        return cv::Point2f(-1.0f, -1.0f);
    }

    return cv::Point2f(
        static_cast<float>(imagePoints[0].x),
        static_cast<float>(imagePoints[0].y)
    );
}

PredictionResult MotionPredictor::predict(
    const SelectionResult& selection,
    const std::vector<std::unique_ptr<Armor>>& armors,
    double currentTime,
    const cv::Vec3d& cameraWorldPosition,
    double yawDegrees,
    double pitchDegrees,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    double bulletSpeed
)
{
    PredictionResult result;

    if(selection.state == SelectionState::NONE ||
       selection.index < 0 ||
       selection.index >= static_cast<int>(armors.size()))
    {
        return result;
    }

    const Armor& armor = *armors[selection.index];
    const cv::Vec3d& currentPositionCamera = armor.pose().tvec;
    result.positionCamera = currentPositionCamera;
    result.hitPoint = armor.getCenter();

    if(armor.number().value() == 8)
    {
        reset();
        return result;
    }

    if(selection.state == SelectionState::FIRST_SELECT ||
       selection.state == SelectionState::SWITCH ||
       selection.state == SelectionState::REACQUIRE)
    {
        reset();
    }

    if(!armor.pose().valid ||
       !isFinitePosition(currentPositionCamera) ||
       currentPositionCamera[2] <= 0.0 ||
       !isFinitePosition(cameraWorldPosition) ||
       !std::isfinite(currentTime) ||
       !std::isfinite(yawDegrees) ||
       !std::isfinite(pitchDegrees))
    {
        reset();
        return result;
    }

    cv::Vec3d currentWorldPosition =
        cameraToReference(currentPositionCamera, yawDegrees, pitchDegrees)
        + cameraWorldPosition;

    if(!isFinitePosition(currentWorldPosition))
    {
        reset();
        return result;
    }

    constexpr double outpostHeightJump = 133.333333 / 2.0;
    if(armor.number().value() == 6 && m_hasLastWorldPosition &&
       std::abs(currentWorldPosition[1] - m_lastWorldPosition[1]) > outpostHeightJump)
    {
        reset();
    }

    result.positionWorld = currentWorldPosition;
    if(!update(currentWorldPosition, currentTime))
        return result;

    if(!m_velocityValid ||
       !std::isfinite(bulletSpeed) || bulletSpeed <= 0.0)
    {
        return result;
    }

    cv::Vec3d predictedPositionWorld = predict3D(
        currentWorldPosition,
        cv::norm(currentPositionCamera),
        bulletSpeed
    );
    cv::Vec3d predictedPositionCamera = referenceToCamera(
        predictedPositionWorld - cameraWorldPosition,
        yawDegrees,
        pitchDegrees
    );

    if(!isFinitePosition(predictedPositionWorld) ||
       !isFinitePosition(predictedPositionCamera) ||
       predictedPositionCamera[2] <= 0.0)
    {
        return result;
    }

    result.positionWorld = predictedPositionWorld;
    result.positionCamera = predictedPositionCamera;
    result.valid = true;
    result.hitPoint = projectToImage(
        predictedPositionCamera,
        cameraMatrix,
        distCoeffs
    );
    return result;
}
