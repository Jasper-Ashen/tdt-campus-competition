#include"armor.hpp"


void LightBar::Calculate()
{
    cv::Point2f pts[4];
    m_rect.points(pts);
    double edge01=norm(pts[0]-pts[1]);
    double edge12=norm(pts[1]-pts[2]);
    cv::Point2f mid1;
    cv::Point2f mid2; 
    if(edge01>edge12)
    {
        mid1=(pts[1]+pts[2])*0.5f;
        mid2=(pts[0]+pts[3])*0.5f;
    }else{
        mid1=(pts[0]+pts[1])*0.5f;
        mid2=(pts[2]+pts[3])*0.5F;
    }
    if(mid1.y>mid2.y)
    {
        m_bottom=mid1;
        m_top=mid2;
    }else{
        m_bottom=mid2;
        m_top=mid1;
    }
    m_length= cv::norm(m_bottom - m_top);
}

std::array<cv::Point2f, 4> calculateNumberCorner(
    const LightBar& left,
    const LightBar& right
)
{
    std::array<cv::Point2f, 4> corners;
    const cv::Point2f origin =
        (left.rect().center + right.rect().center) * 0.5f;
    const float leftLength = left.length();
    const float rightLength = right.length();

    if(leftLength <= 1e-6f || rightLength <= 1e-6f)
    {
        corners.fill(origin);
        return corners;
    }

    cv::Point2f leftAxis =
        (left.bottom() - left.top()) / leftLength;
    cv::Point2f rightAxis =
        (right.bottom() - right.top()) / rightLength;

    cv::Point2f axis = leftAxis + rightAxis;
    const double axisLength = cv::norm(axis);
    if(axisLength <= 1e-6)
    {
        corners.fill(origin);
        return corners;
    }
    axis /= axisLength;

    auto heightOf = [&](const cv::Point2f& point)
    {
        return (point - origin).dot(axis);
    };
    const std::array<cv::Point2f, 4> endpoints = {
        left.top(), left.bottom(),
        right.top(), right.bottom()
    };

    float topHeight = heightOf(endpoints[0]);
    float bottomHeight = topHeight;
    for(const auto& point : endpoints)
    {
        const float height = heightOf(point);
        topHeight = std::min(topHeight, height);
        bottomHeight = std::max(bottomHeight, height);
    }

    const float padding = 0.5f * std::max(leftLength, rightLength);
    topHeight -= padding;
    bottomHeight += padding;

    const cv::Point2f leftCenter = left.rect().center;
    const cv::Point2f rightCenter = right.rect().center;

    const cv::Point2f lt =
        leftCenter + axis * (topHeight - heightOf(leftCenter));
    const cv::Point2f lb =
        leftCenter + axis * (bottomHeight - heightOf(leftCenter));
    const cv::Point2f rt =
        rightCenter + axis * (topHeight - heightOf(rightCenter));
    const cv::Point2f rb =
        rightCenter + axis * (bottomHeight - heightOf(rightCenter));

    corners[0] = lt + (rt - lt) * 0.18f;
    corners[1] = lb + (rb - lb) * 0.18f;
    corners[2] = lb + (rb - lb) * 0.82f;
    corners[3] = lt + (rt - lt) * 0.82f;
    return corners;
}

std::unique_ptr<Armor> createArmor(
    const ArmorCandidate& candidate,
    int number
)
{
    ArmorSize size =
        number == 1 ? ArmorSize::LARGE : ArmorSize::SMALL;

    switch(number)
    {
    case 1:
        return std::make_unique<Armor1>(
            candidate.left,
            candidate.right,
            candidate.color,
            size
        );

    case 2:
        return std::make_unique<Armor2>(
            candidate.left,
            candidate.right,
            candidate.color,
            size
        );

    case 3:
        return std::make_unique<Armor3>(
            candidate.left,
            candidate.right,
            candidate.color,
            size
        );

    case 4:
        return std::make_unique<Armor4>(
            candidate.left,
            candidate.right,
            candidate.color,
            size
        );

    case 5:
        return std::make_unique<Armor5>(
            candidate.left,
            candidate.right,
            candidate.color,
            size
        );

    case 6:
        return std::make_unique<Armor6>(
            candidate.left,
            candidate.right,
            candidate.color,
            ArmorSize::SMALL
        );

    case 7:
        return std::make_unique<Armor7>(
            candidate.left,
            candidate.right,
            candidate.color,
            ArmorSize::SMALL
        );

    case 8:
        return std::make_unique<Armor8>(
            candidate.left,
            candidate.right,
            candidate.color,
            ArmorSize::SMALL
        );

    default:
        return nullptr;
    }
}
