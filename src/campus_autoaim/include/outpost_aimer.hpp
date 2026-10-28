#pragma once

#include <array>
#include <opencv2/core.hpp>

struct OutpostPredictionResult
{
    bool valid = false;
    bool directionValid = false;

    cv::Vec3d positionWorld{0.0, 0.0, 0.0};

    double flightTime = 0.0;
    double predictionTime = 0.0;
    double currentAngle = 0.0;
    double predictedAngle = 0.0;
    double angularVelocity = 0.0;
};

class OutpostInterceptor
{
public:
    void setCenterWorldXZ(
        double centerX,
        double centerZ
    );

    void reset();

    OutpostPredictionResult predict(
        const cv::Vec3d& currentPositionWorld,
        double currentTime,
        const cv::Vec3d& cameraWorldPosition,
        double bulletSpeed,
        double systemDelay
    );

private:
    struct LayerState
    {
        bool valid = false;
        double angle = 0.0;
        double time = 0.0;
    };

    static double wrapPi(double angle);

    int classifyHeight(double worldY) const;

    void updateDirection(
        int layer,
        double angle,
        double currentTime
    );

    double m_centerX = 0.0;
    double m_centerZ = 0.0;
    bool m_hasCenter = false;

    std::array<LayerState, 3> m_layers;

    int m_directionVotes = 0;
    int m_direction = 0;

    static constexpr double radius = 266.67;

    static constexpr double angularSpeed = 2.51327412287;

    static constexpr std::array<double, 3> heights =
    {
        -434.68,
        -301.34,
        -168.01
    };

    static constexpr double heightTolerance = 50.0;
};