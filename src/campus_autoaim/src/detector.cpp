#include "detector.hpp"
#include <cstddef> // [Cleanup] 显式提供容器索引类型std::size_t

void ArmorDetector::setColorThreshold(int value)
{
    m_lightBarDetector.setColorThreshold(value);
}
void ArmorDetector::setBrightnessThreshold(int value)
{
    m_lightBarDetector.setBrightnessThreshold(value);
}

//////提取灯条轮廓//////
std::vector<std::vector<cv::Point>>  LightBarDetector::findCandidateContours
(const cv::Mat& frame,
    TargetColor color)
{
    //cv::Mat blurred;
    //cv::GaussianBlur(frame, blurred, cv::Size(3, 3), 0);
    std::vector<cv::Mat> channel;
    cv::split(frame,channel);
    cv::Mat difference,colormask,brightmask,intensity;
    //cv::createTrackbar("Threshold","mask",&tBrightThresholdValue,255,nullptr);
    //cv::createTrackbar("Threshold","mask",&ColorthresholdValue，255,nullptr);
    //////算色差/////

    if(color==TargetColor::BLUE)
    {
        cv::subtract(channel[0],channel[2],difference);
    }else{
        cv::subtract(channel[2],channel[0],difference);
    }
    cv::threshold(difference,colormask,m_colorThreshold,255,cv::THRESH_BINARY);

    

    ////////算亮度////////

    cv::max(channel[0],channel[1],intensity);
    cv::max(intensity,channel[2],intensity);
    cv::threshold(intensity,brightmask,m_brightnessThreshold,255,cv::THRESH_BINARY);


    ///////先用膨胀消除灯过曝//////

    cv::Mat kernel =cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3));
    cv::dilate(colormask,colormask,kernel);


    ///////亮度+色差 选择目标////////

    cv::Mat mask;
    cv::bitwise_and(colormask,brightmask,mask);
    cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,kernel);

    ///////找轮廓///////
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    //cv::imshow("mask", mask);

    return contours;
};

bool isVertical(
    const cv::RotatedRect& rect,
    float maxTiltRad
)
{
    cv::Point2f pts[4];
    rect.points(pts);

    cv::Point2f edge01 = pts[1] - pts[0];
    cv::Point2f edge12 = pts[2] - pts[1];

    cv::Point2f longEdge;

    if(cv::norm(edge01) >= cv::norm(edge12))
        longEdge = edge01;
    else
        longEdge = edge12;
    float angle=std::atan2(
        std::abs(longEdge.x),
        std::abs(longEdge.y)
    );
    return angle < maxTiltRad;
}

//////找灯类/////
std::vector<LightBar>  LightBarDetector::findCorrectLightBar
(const std::vector<std::vector<cv::Point>>& contours)
{
    std::vector<LightBar> CandidateLightbars;
    for(const auto& contour : contours)
    {
        if(cv::contourArea(contour)<5.0)continue;
        cv::RotatedRect rect = cv::minAreaRect(contour);
        float width=rect.size.width;
        float height = rect.size.height;
        if(width <= 0 || height <= 0)continue;
        float longSide = std::max(width, height);
        if(longSide<5.0f)continue;
        float shortSide = std::min(width, height);
        float aspectRatio = longSide / shortSide;
        if(isVertical(rect,0.8727f)&&aspectRatio>1.7f)
        {
            CandidateLightbars.emplace_back(rect);
        }
    }
    std::sort(
        CandidateLightbars.begin(),
        CandidateLightbars.end(),
        [](const LightBar& a, const LightBar& b)
        {
            return a.rect().center.x < b.rect().center.x;
        }
    );
    return CandidateLightbars;
};

///////灯类配对成装甲板类///////
std::vector<ArmorCandidate> ArmorDetector::findCandidateArmor
(const std::vector<LightBar>&CandidateLightbars,
TargetColor color)
{
    std::vector<ArmorCandidate> CandidateArmors;
    struct Candidate
    {
        std::size_t left;
        std::size_t right;
        float score;
    };
    std::vector<Candidate> candidates;
    for(std::size_t i=0;i<CandidateLightbars.size();i++)
    {
        for(std::size_t j=i+1;j<CandidateLightbars.size();j++)
        {
            const LightBar& light1 = CandidateLightbars[i];
            const LightBar& light2 = CandidateLightbars[j];

            float len1 = light1.length();
            float len2 = light2.length();

            if(len1 <= 0 || len2 <= 0)continue;

            float average = (len1 + len2) * 0.5f;

            float lengthRatio =
                std::max(len1, len2) /
                std::min(len1, len2);

            cv::Point2f delta =
                light2.rect().center -
                light1.rect().center;

            float separation =
                cv::norm(delta) / average;


            cv::Point2f axis1 =
                (light1.bottom() - light1.top()) / len1;
            cv::Point2f axis2 =
                (light2.bottom() - light2.top()) / len2;
            cv::Point2f axis = axis1 + axis2;


            const float radToDeg=180.0f/static_cast<float>(CV_PI);
            float angle1 =std::atan2(axis1.x, axis1.y) * radToDeg;
            float angle2 =std::atan2(axis2.x, axis2.y) * radToDeg;
            float angleDifference =std::abs(angle1 - angle2);
            if(angleDifference > 25.0f)continue;

            float axisLength = cv::norm(axis);

            if(axisLength <= 0)continue;

            axis /= axisLength;

            float vertical =
                std::abs(delta.dot(axis)) /
                average;

            bool normalPair =
                lengthRatio < 1.8f &&
                vertical < 0.65 &&
                separation > 1.1f &&
                separation < 3.8f;

            bool widePair =
                lengthRatio < 1.3f &&
                vertical < 0.35f &&
                angleDifference < 9.0f &&
                separation >= 3.8f &&
                separation < 4.8f;

            if(normalPair || widePair)
            {
                float score =
                        (lengthRatio - 1.0f)
                        + vertical
                        +angleDifference/25.0f;
                candidates.push_back({i,j,score});
            }
        }
        
    }
    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const Candidate& a, const Candidate& b)
        {
            return a.score < b.score;
        }
    );
    std::vector<bool> used(
        CandidateLightbars.size(),
        false
    );
    for(const auto& candidate : candidates)
    {
        if(used[candidate.left] ||
        used[candidate.right])
            continue;
        used[candidate.left] = true;
        used[candidate.right] = true;
        CandidateArmors.push_back(ArmorCandidate{
            CandidateLightbars[candidate.left],
            CandidateLightbars[candidate.right],
            color
        });
    }
    return CandidateArmors;
}

std::vector<ArmorCandidate> ArmorDetector::detect(const cv::Mat& frame,TargetColor color)
{
    auto contours=m_lightBarDetector.findCandidateContours(frame,color);
    auto CandidateLightbars=m_lightBarDetector.findCorrectLightBar(contours);
    auto CandidateArmor=findCandidateArmor(CandidateLightbars,color);
    return CandidateArmor;
};

