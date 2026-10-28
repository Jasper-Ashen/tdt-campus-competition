#pragma once
#include<opencv2/opencv.hpp>
#include<string>

#include "armor.hpp"

class NumberClassifier
{
private:
    cv::HOGDescriptor m_hog;
    cv::Ptr<cv::ml::SVM> m_svm;

public:
    NumberClassifier(const std::string& modelPath);

    int predict(const cv::Mat& image) const;
};

cv::Mat extractNumberImage(
    const cv::Mat& frame,
    const ArmorCandidate& candidate
);