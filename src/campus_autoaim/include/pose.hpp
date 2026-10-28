#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "armor.hpp"



std::vector<cv::Point3f> buildObjectPoints(const Armor& armor);

std::vector<cv::Point2f> buildImagePoints(
    const Armor& armor
);

PoseResult estimatePose(
    const Armor& armor,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    cv::SolvePnPMethod method
);