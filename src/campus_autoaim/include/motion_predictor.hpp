#pragma once

#include <opencv2/core.hpp>

#include "armor.hpp"
#include"target_selector.hpp"

struct PredictionResult
{
    cv::Vec3d positionWorld{0.0, 0.0, 0.0};
    cv::Vec3d positionCamera{0.0, 0.0, 0.0};
    cv::Point2f hitPoint{-1.0f, -1.0f};
    bool valid = false;
};

class MotionPredictor
{
private:
    enum class TrustState
    {
        EMPTY,
        WARMUP,
        TRUSTED
    };

    TrustState m_trustState = TrustState::EMPTY;
    bool m_hasProvisionalVelocity = false;

    cv::Vec3d m_lastWorldPosition{0.0, 0.0, 0.0};
    cv::Vec3d m_velocityWorld{0.0, 0.0, 0.0};

    double m_lastTime = 0.0;

    bool m_hasLastWorldPosition = false;
    bool m_velocityValid = false;

    void reset();

    bool update(
        const cv::Vec3d& currentWorldPosition,
        double currentTime
    );

    cv::Vec3d predict3D(
        const cv::Vec3d& currentWorldPosition,
        double currentDistance,
        double bulletSpeed
    ) const;

    cv::Point2f projectToImage(
        const cv::Vec3d& position,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs
    ) const;

public:
    MotionPredictor() = default;

    PredictionResult predict(
        const SelectionResult& selection,
        const std::vector<std::unique_ptr<Armor>>& armors,
        double currentTime,
        const cv::Vec3d& cameraWorldPosition,
        double yawDegrees,
        double pitchDegrees,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs,
        double bulletSpeed
    );
};
