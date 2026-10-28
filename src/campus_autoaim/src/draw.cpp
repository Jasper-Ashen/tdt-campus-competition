#include "draw.hpp"
#include <string>
#include <opencv2/imgproc.hpp>

void drawArmors(
    cv::Mat& frame,
    const std::vector<std::unique_ptr<Armor>>& armors,
    int selectedIndex
)
{
    for(size_t i = 0; i < armors.size(); i++)
    {
        Armor& armor = *armors[i];
        cv::Point2f leftPts[4];
        armor.left().rect().points(leftPts);

        for(int i = 0; i < 4; i++)
        {
            cv::line(
                frame,
                leftPts[i],
                leftPts[(i + 1) % 4],
                cv::Scalar(0, 255, 0),
                2
            );
        }

        cv::Point2f rightPts[4];
        armor.right().rect().points(rightPts);

        for(int i = 0; i < 4; i++)
        {
            cv::line(
                frame,
                rightPts[i],
                rightPts[(i + 1) % 4],
                cv::Scalar(0, 255, 0),
                2
            );
        }

        cv::circle(
            frame,
            armor.getCenter(),
            4,
            cv::Scalar(0, 255, 255),
            -1
        );

        if(static_cast<int>(i) == selectedIndex)
        {
            cv::Scalar selectedColor(0, 0, 255);

            cv::Point2f p1 = armor.left().top();
            cv::Point2f p2 = armor.left().bottom();
            cv::Point2f p3 = armor.right().bottom();
            cv::Point2f p4 = armor.right().top();

            cv::line(frame, p1, p2, selectedColor, 4);
            cv::line(frame, p2, p3, selectedColor, 4);
            cv::line(frame, p3, p4, selectedColor, 4);
            cv::line(frame, p4, p1, selectedColor, 4);
            continue;
        }

        const auto& corners = armor.numberCorner();

        for(int i = 0; i < 4; i++)
        {
            cv::line(
                frame,
                corners[i],
                corners[(i + 1) % 4],
                cv::Scalar(255, 0, 255),
                2
            );
        }

        const Number& number = armor.number();
        if(number.valid())
        {
            cv::putText(
                frame,
                std::to_string(number.value()),
                number.position(),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(0, 255, 255),
                2
            );
        }
    }
}

void drawFPS(
    cv::Mat& frame,
    int64& prevTick
)
{
    int64 currentTick = cv::getTickCount();
    double elapsed =
        (currentTick - prevTick)/ cv::getTickFrequency();
    double fps = 1.0 / elapsed;
    prevTick = currentTick;
    std::string fpsText =
        cv::format("FPS: %.1f", fps);
    cv::putText(
        frame,
        fpsText,
        cv::Point(frame.cols - 200, 40),
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        cv::Scalar(255,255,255),
        2
    );
}

void drawPoseInfo(
    cv::Mat& frame,
    const Armor& armor
)
{
    const PoseResult& pose = armor.pose();

    if(!pose.valid)
        return;

    std::vector<std::string> texts = {
        cv::format("x: %.1f mm", pose.tvec[0]),
        cv::format("y: %.1f mm", pose.tvec[1]),
        cv::format("z: %.1f mm", pose.tvec[2]),
        cv::format("d: %.1f mm", pose.distance),
        cv::format("yaw: %.1f deg", pose.yaw),
        cv::format("pitch: %.1f deg", pose.pitch),
        cv::format("roll: %.1f deg", pose.roll)
    };

    cv::Point position(
        cvRound(armor.getCenter().x + 10),
        cvRound(armor.getCenter().y - 60)
    );

    const int lineSpacing = 18;

    for(size_t i = 0; i < texts.size(); i++)
    {
        cv::putText(
            frame,
            texts[i],
            position + cv::Point(
                0,
                static_cast<int>(i) * lineSpacing
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 255),
            1
        );
    }
}