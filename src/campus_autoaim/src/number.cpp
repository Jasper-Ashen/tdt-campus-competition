#include "number.hpp"
#include <cstddef> // [Cleanup] 显式提供容器索引类型std::size_t
#include <stdexcept>
#include <vector>

//////构造数字分类器/////
NumberClassifier::NumberClassifier(
    const std::string& modelPath
)
{
    m_hog = cv::HOGDescriptor(
        cv::Size(20, 28),
        cv::Size(8, 8),
        cv::Size(4, 4),
        cv::Size(4, 4),
        9
    );

    m_svm = cv::ml::SVM::load(modelPath);
    if(m_svm.empty())
    {
        throw std::runtime_error(
            "failed to load SVM model"
        );
    }
}

///////提取数字区域灰度图///////
cv::Mat extractNumberImage(
    const cv::Mat& frame,
    const ArmorCandidate& candidate
)
{
    auto corners = calculateNumberCorner(
        candidate.left,
        candidate.right
    );

    const int numberWidth = 20;
    const int numberHeight = 28;
    std::array<cv::Point2f, 4> dstPoints = {
        cv::Point2f(0, 0),
        cv::Point2f(0, numberHeight - 1),
        cv::Point2f(numberWidth - 1, numberHeight - 1),
           cv::Point2f(numberWidth - 1, 0)
    };
    cv::Mat transformMatrix =
        cv::getPerspectiveTransform(
            corners.data(),
            dstPoints.data()
        );
    cv::Mat numberImage;
    cv::warpPerspective(
        frame,
        numberImage,
        transformMatrix,
        cv::Size(numberWidth, numberHeight)
    );
    cv::Mat grayNumberImage;

    cv::cvtColor(
        numberImage,
        grayNumberImage,
        cv::COLOR_BGR2GRAY
    );
    return grayNumberImage;
}

/////接受灰度裁取图片，并预测数字///// 
int NumberClassifier::predict(const cv::Mat& image) const
{
    std::vector<float> descriptor;
    m_hog.compute(
        image,
        descriptor
    );
    cv::Mat sample(
        1,
        static_cast<int>(descriptor.size()),
        CV_32F
    );
    for(std::size_t j = 0; j < descriptor.size(); j++)
    {
        sample.at<float>(0, static_cast<int>(j)) = descriptor[j];
    }
    float prediction =m_svm->predict(sample);
    int number =static_cast<int>(prediction);
    return number;
}