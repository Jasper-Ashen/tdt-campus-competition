#pragma once
#include <memory>
#include <vector>
#include <opencv2/core.hpp>

#include "armor.hpp"

void drawArmors(
    cv::Mat& frame,
    const std::vector<std::unique_ptr<Armor>>& armors,
    int selectedIndex
);

void drawFPS(
    cv::Mat& frame,
    int64& prevTick
);

void drawPoseInfo(
    cv::Mat& frame,
    const Armor& armor
);
