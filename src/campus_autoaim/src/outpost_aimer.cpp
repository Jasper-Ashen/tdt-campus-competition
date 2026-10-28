#include "outpost_aimer.hpp"

#include <cmath>
#include <algorithm>

void OutpostInterceptor::setCenterWorldXZ(
    double centerX,
    double centerZ
)
{
    m_centerX = centerX;
    m_centerZ = centerZ;

    m_hasCenter =
        std::isfinite(centerX) &&
        std::isfinite(centerZ);

    reset();
}

void OutpostInterceptor::reset()
{
    for(auto& layer : m_layers)
    {
        layer = LayerState{};
    }

    m_directionVotes = 0;
    m_direction = 0;
}

double OutpostInterceptor::wrapPi(double angle)
{
    while(angle > CV_PI)
        angle -= 2.0 * CV_PI;

    while(angle <= -CV_PI)
        angle += 2.0 * CV_PI;

    return angle;
}

int OutpostInterceptor::classifyHeight(
    double worldY
) const
{
    int bestLayer = -1;
    double bestError = heightTolerance;

    for(int i = 0; i < 3; ++i)
    {
        double error =
            std::abs(worldY - heights[i]);

        if(error < bestError)
        {
            bestError = error;
            bestLayer = i;
        }
    }

    return bestLayer;
}

void OutpostInterceptor::updateDirection(
    int layer,
    double angle,
    double currentTime
)
{
    if(layer < 0 || layer >= 3)return;

    LayerState& state = m_layers[layer];

    if(!state.valid)
    {
        state.valid = true;
        state.angle = angle;
        state.time = currentTime;
        return;
    }

    double dt =
        currentTime - state.time;

    if(!std::isfinite(dt) ||
       dt <= 0.01 ||
       dt > 0.30)
    {
        state.angle = angle;
        state.time = currentTime;
        return;
    }

    double delta =
        wrapPi(angle - state.angle);

    state.angle = angle;
    state.time = currentTime;

    double expected =
        angularSpeed * dt;

    double positiveError =
        std::abs(delta - expected);

    double negativeError =
        std::abs(delta + expected);

    constexpr double maxError = 0.25;

    if(std::min(
           positiveError,
           negativeError
       ) > maxError)
    {
        return;
    }

    if(positiveError < negativeError)
        m_directionVotes++;
    else
        m_directionVotes--;

    m_directionVotes = std::clamp(m_directionVotes,-5,5);

    if(m_directionVotes >= 3)
        m_direction = 1;
    else if(m_directionVotes <= -3)
        m_direction = -1;
}

OutpostPredictionResult
OutpostInterceptor::predict(
    const cv::Vec3d& currentPositionWorld,
    double currentTime,
    const cv::Vec3d& cameraWorldPosition,
    double bulletSpeed,
    double systemDelay
)
{
    OutpostPredictionResult result;

    if(!m_hasCenter ||
       !std::isfinite(currentTime) ||
       !std::isfinite(bulletSpeed) ||
       bulletSpeed <= 0.0)
    {
        return result;
    }

    int layer =
        classifyHeight(
            currentPositionWorld[1]
        );

    if(layer < 0)
        return result;

    double dx =
        currentPositionWorld[0] - m_centerX;

    double dz =
        currentPositionWorld[2] - m_centerZ;

    double currentAngle =
        std::atan2(dz, dx);

    updateDirection(
        layer,
        currentAngle,
        currentTime
    );

    result.currentAngle = currentAngle;

    result.directionValid = m_direction != 0;

    if(m_direction == 0)return result;

    double omega = static_cast<double>(m_direction) * angularSpeed;

    double flightTime = cv::norm(currentPositionWorld - cameraWorldPosition) / bulletSpeed;

    double predictionTime = flightTime + systemDelay;

    cv::Vec3d predictedWorld;

    double predictedAngle =
        currentAngle;

    for(int i = 0; i < 2; ++i)
    {
        predictedAngle = currentAngle + omega * predictionTime;

        predictedWorld =
            cv::Vec3d(
                m_centerX +
                    radius *
                    std::cos(predictedAngle),

                heights[layer],

                m_centerZ +
                    radius *
                    std::sin(predictedAngle)
            );

        flightTime =
            cv::norm(
                predictedWorld -
                cameraWorldPosition
            ) / bulletSpeed;

        predictionTime =
            flightTime + systemDelay;
    }

    predictedAngle = currentAngle + omega * predictionTime;

    predictedWorld =
        cv::Vec3d(
            m_centerX +
                radius *
                std::cos(predictedAngle),

            heights[layer],

            m_centerZ +
                radius *
                std::sin(predictedAngle)
        );

    result.valid = true;
    result.positionWorld = predictedWorld;

    result.flightTime = flightTime;
    result.predictionTime = predictionTime;

    result.predictedAngle =
        predictedAngle;

    result.angularVelocity = omega;

    return result;
}