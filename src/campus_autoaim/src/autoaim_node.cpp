#include <cstdint>
#include <algorithm> 
#include <memory>
#include <cmath>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tdt_interface/msg/auto_aim_frame.hpp"
#include "tdt_interface/msg/send_data.hpp"

#include "detector.hpp"
#include "armor.hpp"
#include "number.hpp"
#include "draw.hpp"
#include "pose.hpp"
#include "target_selector.hpp"
#include "motion_predictor.hpp"
#include "aim_solver.hpp"
#include "gimbal_controller.hpp"
#include "outpost_aimer.hpp"

std::unique_ptr<Armor> createDetectedArmor(
    const cv::Mat& frame,
    const ArmorCandidate& candidate,
    NumberClassifier& classifier,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs
)
{
    cv::Mat grayNumberImage =
        extractNumberImage(frame, candidate);

    int number =
        classifier.predict(grayNumberImage);

    auto armor =
        createArmor(candidate, number);

    if(!armor)
        return nullptr;

    PoseResult pose = estimatePose(
        *armor,
        cameraMatrix,
        distCoeffs,
        cv::SOLVEPNP_IPPE
    );

    armor->setPose(pose);

    return armor;
}

class AutoAimNode : public rclcpp::Node
{
public:
    AutoAimNode()
        : Node("autoaim_node"),
          classifier(
              ament_index_cpp::get_package_share_directory(
                  "campus_autoaim"
              ) + "/model/svm_model.yml"
          )
    {
        blueDetector.setColorThreshold(45);
        blueDetector.setBrightnessThreshold(184);
        redDetector.setColorThreshold(70);
        redDetector.setBrightnessThreshold(184);


        if(targetColor == TargetColor::RED)
        {
            outpostInterceptor.setCenterWorldXZ(
                -5501.43,
                3056.09
            );
        }
        else
        {
            outpostInterceptor.setCenterWorldXZ(
                2212.74,
                -3056.09
            );
        }

        subscription =
            create_subscription<tdt_interface::msg::AutoAimFrame>(
                "/autoaim/frame",
                rclcpp::QoS(1).reliable().durability_volatile(),
                [this](
                    tdt_interface::msg::AutoAimFrame::ConstSharedPtr message
                )
                {
                    receiveFrame(message);
                }
            );

        aimPublisher =
            create_publisher<tdt_interface::msg::SendData>(
                targetColor == TargetColor::RED
                ? "/target_angles_player_1"
                : "/target_angles_player_2",
                rclcpp::QoS(1).reliable().durability_volatile()
            );

        RCLCPP_INFO(
            get_logger(),
            "Waiting for /autoaim/frame"
        );
    }

private:
    void receiveFrame(
        tdt_interface::msg::AutoAimFrame::ConstSharedPtr message
    )
    {
        receivedCount++;

        double currentTime =
            static_cast<double>(message->header.stamp.sec)
            + static_cast<double>(message->header.stamp.nanosec) * 1e-9;

        BuildingAttackPermissions buildingPermissions = buildingRules.update(
            message->state.enemy_healths,
            currentTime,
            message->state.remaining_seconds
        );

        bool combatUnavailable =
            message->state.health <= 0 ||
            message->state.ammo_remaining <= 0;

        if(combatUnavailable)
        {
            if(!m_combatPaused)
            {
                selector = TargetSelector{};
                predictor = MotionPredictor{};
                outpostInterceptor.reset();
            }
            m_combatPaused = true;

            GimbalCommand searchCommand = gimbalController.update(
                false,
                false,
                0.0,
                0.0,
                0.0,
                -1,
                message->state.yaw_degrees,
                message->state.pitch_degrees,
                message->state.twist.linear.x,
                message->state.twist.linear.y,
                currentTime
            );

            tdt_interface::msg::SendData command;
            command.yaw = static_cast<float>(searchCommand.yaw);
            command.pitch = static_cast<float>(searchCommand.pitch);
            command.if_shoot = false;
            aimPublisher->publish(command);
        }
        else
        {
            m_combatPaused = false;
        }

        if(message->compressed_image.data.empty())
        {
            return;
        }

        cv::Mat frame = cv::imdecode(
            message->compressed_image.data,
            cv::IMREAD_COLOR
        );

        if(frame.empty())
        {
            return;
        }

        if(m_combatPaused)
        {
            drawFPS(frame,prevTick);
            cv::imshow("campus_camera", frame);
            if(cv::waitKey(1) == 27)
            {
                rclcpp::shutdown();
            }
            return;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "received=%llu, sequence=%llu, size=%ux%u",
            static_cast<unsigned long long>(receivedCount),
            static_cast<unsigned long long>(
                message->state.frame_sequence
            ),
            message->camera_info.width,
            message->camera_info.height
        );

        const auto& cameraInfo = message->camera_info;

        if(cameraInfo.width != static_cast<std::uint32_t>(frame.cols) ||
           cameraInfo.height != static_cast<std::uint32_t>(frame.rows) ||
           !std::isfinite(cameraInfo.k[0]) ||
           !std::isfinite(cameraInfo.k[4]) ||
           cameraInfo.k[0] <= 0.0 || cameraInfo.k[4] <= 0.0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Invalid or mismatched camera_info; skipping this frame"
            );
            return;
        }

        cv::Mat frameCameraMatrix(3, 3, CV_64F);
        for(int i = 0; i < 9; i++)
        {
            frameCameraMatrix.at<double>(i / 3, i % 3) =
                cameraInfo.k[i];
        }

        cv::Mat frameDistCoeffs;
        if(!cameraInfo.d.empty())
        {
            frameDistCoeffs = cv::Mat(
                1,
                static_cast<int>(cameraInfo.d.size()),
                CV_64F
            );
            for(size_t i = 0; i < cameraInfo.d.size(); i++)
            {
                frameDistCoeffs.at<double>(0, static_cast<int>(i)) =
                    cameraInfo.d[i];
            }
        }

        ArmorDetector& detector = targetColor == TargetColor::RED ? redDetector : blueDetector;

        auto candidates = detector.detect(frame, targetColor);

        std::vector<std::unique_ptr<Armor>> armors;

        for(const auto& candidate : candidates)
        {
            auto armor = createDetectedArmor(
                frame,
                candidate,
                classifier,
                frameCameraMatrix,
                frameDistCoeffs
            );

            if(!armor)continue;

            armors.push_back(std::move(armor));
        }

        SelectionResult selection = selector.select(armors,targetColor,buildingPermissions); 

        int selectedIndex = selection.index;

        AimResult aimResult;
        bool targetValid = false;
        double targetDistance = 0.0;
        int targetNumber = -1;
        bool outpostAimed = false;
        bool outpostInside = false;
        double outpostFacingAngle = -1.0;
        constexpr double outpostMaxFacingAngle = 45.0; 
        cv::Point2f crosshair(
            static_cast<float>(frameCameraMatrix.at<double>(0, 2)),
            static_cast<float>(frameCameraMatrix.at<double>(1, 2))
        );
        double bulletSpeed =
            static_cast<double>(message->state.projectile_speed) * 1000.0 / 3.0;

        if(!std::isfinite(bulletSpeed) ||
        bulletSpeed <= 0.0)
        {
            bulletSpeed = 25000.0;
        }

        cv::Vec3d cameraWorldPosition = cameraWorldPositionFromMap(
            cv::Vec3d(
                message->state.position.x,
                message->state.position.y,
                message->state.position.z
            ),
            message->state.yaw_degrees,
            message->state.pitch_degrees
        );

        PredictionResult prediction = predictor.predict(
            selection,
            armors,
            currentTime,
            cameraWorldPosition,
            message->state.yaw_degrees,
            message->state.pitch_degrees,
            frameCameraMatrix,
            frameDistCoeffs,
            bulletSpeed
        );

        if(selectedIndex >= 0 &&
          selectedIndex < static_cast<int>(armors.size()))
        {
            const Armor& selectedArmor = *armors[selectedIndex];
            targetNumber = selectedArmor.number().value();
            const PoseResult& selectedPose = selectedArmor.pose();

            cv::Vec3d aimPosition;

            if(selectedArmor.number().value() == 6)
            {
                auto imagePoints = buildImagePoints(selectedArmor);
                bool pointsValid = std::all_of(
                    imagePoints.begin(),
                    imagePoints.end(),
                    [](const cv::Point2f& point)
                    {
                        return std::isfinite(point.x) && std::isfinite(point.y);
                    }
                );
                if(pointsValid && std::isfinite(crosshair.x) && std::isfinite(crosshair.y) &&
                   cv::isContourConvex(imagePoints) && std::abs(cv::contourArea(imagePoints)) > 1e-6)
                {
                    outpostInside = cv::pointPolygonTest(imagePoints, crosshair, false) >= 0.0;
                }

                bool poseFinite = selectedPose.valid;
                for(int i = 0; i < 3; i++)
                {
                    poseFinite = poseFinite && std::isfinite(selectedPose.rvec[i]) &&
                        std::isfinite(selectedPose.tvec[i]);
                }
                double distance = cv::norm(selectedPose.tvec);
                if(poseFinite && std::isfinite(distance) && distance > 0.0 && selectedPose.tvec[2] > 0.0)
                {
                    cv::Mat rotationMatrix;
                    cv::Rodrigues(selectedPose.rvec, rotationMatrix);
                    cv::Vec3d normal(
                        rotationMatrix.at<double>(0, 2),
                        rotationMatrix.at<double>(1, 2),
                        rotationMatrix.at<double>(2, 2)
                    );
                    cv::Vec3d viewDirection = selectedPose.tvec / distance;
                    double facingCos = std::clamp(std::abs(normal.dot(viewDirection)), 0.0, 1.0);
                    outpostFacingAngle = std::acos(facingCos) * 180.0 / CV_PI;
                    outpostAimed = outpostInside && std::isfinite(outpostFacingAngle) &&
                        outpostFacingAngle <= outpostMaxFacingAngle;
                }

                cv::Vec3d currentPositionWorld =
                    cameraToReference(
                        selectedPose.tvec,
                        message->state.yaw_degrees,
                        message->state.pitch_degrees
                    )
                    + cameraWorldPosition;

                double systemDelay =
                    static_cast<double>(
                        message->capture_to_publish_ms
                    ) / 1000.0;

                OutpostPredictionResult outpostPrediction =
                    outpostInterceptor.predict(
                        currentPositionWorld,
                        currentTime,
                        cameraWorldPosition,
                        bulletSpeed,
                        systemDelay
                    );

                if(outpostPrediction.valid)
                {
                    cv::Vec3d relativeWorld =
                        outpostPrediction.positionWorld -
                        cameraWorldPosition;

                    aimPosition =
                        referenceToCamera(
                            relativeWorld,
                            message->state.yaw_degrees,
                            message->state.pitch_degrees
                        );

                    targetDistance =
                        cv::norm(relativeWorld);
                }
                else
                {
                    aimPosition =
                        selectedPose.tvec;

                    targetDistance =
                        selectedPose.distance;
                }
            }
            else
            {
                aimPosition =
                    prediction.valid
                    ? prediction.positionCamera
                    : selectedPose.tvec;

                targetDistance =
                    selectedPose.distance;
            }

            aimResult =
                solveAim(
                    aimPosition,
                    message->state.yaw_degrees,
                    message->state.pitch_degrees,
                    bulletSpeed
                );

            targetValid =
                aimResult.valid;

            aimResult =
                solveAim(
                    aimPosition,
                    message->state.yaw_degrees,
                    message->state.pitch_degrees,
                    bulletSpeed
                );

            targetValid = aimResult.valid;

        }

        GimbalCommand gimbalCommand =
            gimbalController.update(
                selector.isLocked(),
                targetValid,
                aimResult.yaw,
                aimResult.pitch,
                targetDistance,
                targetNumber,
                message->state.yaw_degrees,
                message->state.pitch_degrees,
                message->state.twist.linear.x,
                message->state.twist.linear.y,
                currentTime,
                outpostAimed 
            );

        if(gimbalCommand.valid)
        {
            tdt_interface::msg::SendData command;

            command.yaw =
                static_cast<float>(gimbalCommand.yaw);

            command.pitch =
                static_cast<float>(gimbalCommand.pitch);

            command.if_shoot = gimbalCommand.shoot;

            aimPublisher->publish(command);
        }

        cv::Point2f hitPoint = prediction.hitPoint;

        if(hitPoint.x >= 0.0f &&
          hitPoint.y >= 0.0f)
        {
            cv::circle(
                frame,
                hitPoint,
                6,
                cv::Scalar(255, 255, 0),
                2
            );
        }

        drawArmors(frame, armors,selectedIndex);

        for(const auto& armor : armors)
        {
            drawPoseInfo(frame, *armor);
        }

        if(targetNumber == 6 && std::isfinite(crosshair.x) && std::isfinite(crosshair.y))
        {
            cv::drawMarker(frame, crosshair, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 18, 2);
            cv::putText(
                frame,
                cv::format("Outpost inside=%d face=%.1f/%.0f shoot=%d",
                    outpostInside, outpostFacingAngle, outpostMaxFacingAngle, gimbalCommand.shoot),
                cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                outpostAimed ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255),
                2
            );
        }

        // FPS
        drawFPS(frame,prevTick);

        cv::imshow("campus_camera", frame);
        if(cv::waitKey(1) == 27)
        {
            rclcpp::shutdown();
        }
    }

    rclcpp::Subscription<
        tdt_interface::msg::AutoAimFrame
    >::SharedPtr subscription;

    rclcpp::Publisher<
        tdt_interface::msg::SendData
    >::SharedPtr aimPublisher;

    TargetColor targetColor = TargetColor::RED;

    ArmorDetector blueDetector;
    ArmorDetector redDetector;
    TargetSelector selector;
    BuildingAttackRules buildingRules; 
    MotionPredictor predictor;
    OutpostInterceptor outpostInterceptor;


    NumberClassifier classifier;
    GimbalController gimbalController;

    int64 prevTick = cv::getTickCount();
    std::uint64_t receivedCount = 0;
    bool m_combatPaused = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<AutoAimNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
