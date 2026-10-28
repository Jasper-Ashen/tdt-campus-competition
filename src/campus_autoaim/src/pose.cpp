#include "pose.hpp"

std::vector<cv::Point3f> buildObjectPoints(const Armor& armor)
{
    float armorWidth;

    if(armor.getSize() == ArmorSize::SMALL)
    {
        armorWidth = 135.0f;
    }
    else
    {
        armorWidth = 230.0f;
    }
    
    constexpr float lightLength = 55.0f;

    float halfWidth = armorWidth * 0.5f;
    float halfHeight = lightLength * 0.5f;

    return {
        {-halfWidth, -halfHeight, 0.0f},
        {-halfWidth,  halfHeight, 0.0f},
        { halfWidth,  halfHeight, 0.0f},
        { halfWidth, -halfHeight, 0.0f}
    };
}

std::vector<cv::Point2f> buildImagePoints(
    const Armor& armor
)
{
    return {
        armor.left().top(),
        armor.left().bottom(),
        armor.right().bottom(),
        armor.right().top()
    };
}

PoseResult estimatePose(
    const Armor& armor,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    cv::SolvePnPMethod method
)
{
    PoseResult result;

    auto objectPoints = buildObjectPoints(armor);
    auto imagePoints = buildImagePoints(armor);

    bool success = cv::solvePnP(
        objectPoints,
        imagePoints,
        cameraMatrix,
        distCoeffs,
        result.rvec,
        result.tvec,
        false,
        method
    );

    if(success)
    {
        result.distance = cv::norm(result.tvec);

        cv::Mat rotationMatrix;

        cv::Rodrigues(
            result.rvec,
            rotationMatrix
        );

        double R00 = rotationMatrix.at<double>(0, 0);
        double R10 = rotationMatrix.at<double>(1, 0);
        double R20 = rotationMatrix.at<double>(2, 0);
        double R21 = rotationMatrix.at<double>(2, 1);
        double R22 = rotationMatrix.at<double>(2, 2);

        double yaw =
            std::atan2(R10, R00);

        double pitch =
            std::asin(-R20);

        double roll =
            std::atan2(R21, R22);

        const double radToDeg = 180.0 / CV_PI;

        result.yaw = yaw * radToDeg;
        result.pitch = pitch * radToDeg;
        result.roll = roll * radToDeg;
        
        result.valid = true;
    }

    return result;
}