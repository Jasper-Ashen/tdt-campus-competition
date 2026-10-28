#pragma once
#include<opencv2/opencv.hpp>
#include<vector>
#include<armor.hpp>
#include<cmath>




class LightBarDetector
{
private:
    int m_colorThreshold = 45;
    int m_brightnessThreshold = 184;
public:
    std::vector<LightBar>  findCorrectLightBar
    (const std::vector<std::vector<cv::Point>>& contours);

    std::vector<std::vector<cv::Point>> findCandidateContours
        (const cv::Mat& frame,TargetColor color);


    void setColorThreshold(int value)
    {
        m_colorThreshold = value;
    }

    void setBrightnessThreshold(int value)
    {
        m_brightnessThreshold = value;
    }

};



class  ArmorDetector
{
private:
    LightBarDetector m_lightBarDetector;
public:
    std::vector<ArmorCandidate> detect(const cv::Mat& frame,TargetColor color);
public:
    void setColorThreshold(int value);
    void setBrightnessThreshold(int value);
    std::vector<ArmorCandidate> findCandidateArmor
    (const std::vector<LightBar>&CandidateLightbars,
    TargetColor color);
};