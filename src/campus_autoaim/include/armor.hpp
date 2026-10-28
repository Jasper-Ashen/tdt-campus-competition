#pragma once
#include<opencv2/opencv.hpp>
#include<algorithm>
#include<array>
#include<memory>

///////两种颜色///////
enum class TargetColor
{
    BLUE,
    RED
};

///////两种型号//////
enum class ArmorSize
{
    SMALL,
    LARGE
};

///////灯管类///////
class LightBar
{
private:
    cv::RotatedRect m_rect;
    cv::Point2f m_top;
    cv::Point2f m_bottom;
    float m_length;
    void Calculate();
public:
    /////初始化////

    LightBar(
        cv::RotatedRect rect
    )
    :m_rect(rect)
    {
        Calculate();
    }

    //////读取函数
    const cv::RotatedRect& rect() const
    {
        return m_rect;
    }

    const cv::Point2f& top() const
    {
        return m_top;
    }

    const cv::Point2f& bottom() const
    {
        return m_bottom;
    }

    float length() const
    {
        return m_length;
    }
};

///////候选装甲板//////
struct ArmorCandidate
{
    LightBar left;
    LightBar right;
    TargetColor color;
};

//////装甲板上数字/////
class Number
{
private:
    int m_value = -1;
    cv::Point m_position;

public:
    Number() = default;

    explicit Number(int value)
        : m_value(value) {}

    void setValue(int value)
    {
        m_value = value;
    }

    int value() const
    {
        return m_value;
    }

    void setPosition(const cv::Point& position)
    {
        m_position = position;
    }

    const cv::Point& position() const
    {
        return m_position;
    }

    bool valid() const
    {
        return m_value >= 0;
    }

};

std::array<cv::Point2f, 4> calculateNumberCorner(
    const LightBar& left,
    const LightBar& right
);

////位姿数据////
struct PoseResult
{
    cv::Vec3d rvec{0.0, 0.0, 0.0};
    cv::Vec3d tvec{0.0, 0.0, 0.0};

    double distance = 0.0;

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    bool valid = false;
};

//////装甲板类///////
class Armor
{
private:
    LightBar m_left;
    LightBar m_right;
    std::array<cv::Point2f, 4> m_numberCorner;
    cv::Point2f center;
    TargetColor color;
    Number m_num;
    PoseResult m_pose;
    ArmorSize m_size;

public:
    Armor(
        const LightBar& left,
        const LightBar& right,
        TargetColor armorColor,
        ArmorSize armorSize
    )
        : m_left(left),
        m_right(right),
        color(armorColor),
        m_size(armorSize)
    {
        center =
            (m_left.rect().center + m_right.rect().center) * 0.5f;
        m_numberCorner = calculateNumberCorner(m_left, m_right);
        m_num.setPosition(cv::Point(
            cvRound(m_numberCorner[1].x),
            cvRound(m_numberCorner[1].y)
        ));
    }

    virtual ~Armor() = default;

    const LightBar& left() const
    {
        return m_left;
    }

    const LightBar& right() const
    {
        return m_right;
    }

    const std::array<cv::Point2f, 4>& numberCorner() const
    {
        return m_numberCorner;
    }

    const cv::Point2f& getCenter() const
    {
        return center;
    }

    TargetColor getColor() const
    {
        return color;
    }

    void setNumberValue(int value)
    {
        m_num.setValue(value);
    }

    const Number& number() const
    {
        return m_num;
    }

    void setPose(const PoseResult& pose)
    {
        m_pose = pose;
    }

    const PoseResult& pose() const
    {
        return m_pose;
    }

    ArmorSize getSize() const
    {
        return m_size;
    }

};

class Armor1 : public Armor
{
public:
    Armor1(
        const LightBar& left,
        const LightBar& right,
        TargetColor color,
        ArmorSize size
    )
        : Armor(left, right, color,size)
    {
        setNumberValue(1);
    }
};

class Armor2 : public Armor
{
public:
    Armor2(
        const LightBar& left,
        const LightBar& right,
        TargetColor color,
        ArmorSize size
    )
        : Armor(left, right, color,size)
    {
        setNumberValue(2);
    }
};

class Armor3 : public Armor
{
public:
    Armor3(
        const LightBar& left,
        const LightBar& right,
        TargetColor color,
        ArmorSize size
    )
        : Armor(left, right, color,size)
    {
        setNumberValue(3);
    }
};

class Armor4 : public Armor
{
public:
    Armor4(
        const LightBar& left,
        const LightBar& right,
        TargetColor color,
        ArmorSize size
    )
        : Armor(left, right, color,size)
    {
        setNumberValue(4);
    }
};

class Armor5 : public Armor
{
public:
    Armor5(
        const LightBar& left,
        const LightBar& right,
        TargetColor color,
        ArmorSize size
    )
        : Armor(left, right, color,size)
    {
        setNumberValue(5);
    }
};

class Armor6 : public Armor
{
public:
    Armor6(
        const LightBar& left,
        const LightBar& right,
        TargetColor color,
        ArmorSize size
    )
        : Armor(left, right, color,size)
    {
        setNumberValue(6);
    }
};

class Armor7 : public Armor
{
public:
    Armor7(
        const LightBar& left,
        const LightBar& right,
        TargetColor color,
        ArmorSize size
    )
        : Armor(left, right, color,size)
    {
        setNumberValue(7);
    }
};

class Armor8 : public Armor
{
public:
    Armor8(
        const LightBar& left,
        const LightBar& right,
        TargetColor color,
        ArmorSize size
    )
        : Armor(left, right, color,size)
    {
        setNumberValue(8);
    }
};

std::unique_ptr<Armor> createArmor(
    const ArmorCandidate& candidate,
    int number
);


