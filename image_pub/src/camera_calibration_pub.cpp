#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/distortion_models.hpp"

using namespace std::chrono_literals;


class CameraCalibrationPub : public rclcpp::Node
{
    public:
        CameraCalibrationPub()
        : Node("camera_info_publisher")
        {

            this->declare_parameter("camera/left/fx", 0.0);
            this->declare_parameter("camera/left/fy", 0.0);
            this->declare_parameter("camera/left/cx", 0.0);
            this->declare_parameter("camera/left/cy", 0.0);
            this->declare_parameter("camera/left/k1", 0.0);
            this->declare_parameter("camera/left/k2", 0.0);
            this->declare_parameter("camera/left/p1", 0.0);
            this->declare_parameter("camera/left/p2", 0.0);
            this->declare_parameter("camera/left/width", 0);
            this->declare_parameter("camera/left/height", 0);
            this->declare_parameter("camera/left/r12", 0.0);
            this->declare_parameter("camera/left/r13", 0.0);
            this->declare_parameter("camera/left/r21", 0.0);
            this->declare_parameter("camera/left/r23", 0.0);
            this->declare_parameter("camera/left/r31", 0.0);
            this->declare_parameter("camera/left/r32", 0.0);
            this->declare_parameter("camera/left/fx_p", 0.0);
            this->declare_parameter("camera/left/fy_p", 0.0);
            this->declare_parameter("camera/left/cx_p", 0.0);
            this->declare_parameter("camera/left/cy_p", 0.0);

            this->declare_parameter("camera/right/fx", 0.0);
            this->declare_parameter("camera/right/fy", 0.0);
            this->declare_parameter("camera/right/cx", 0.0);
            this->declare_parameter("camera/right/cy", 0.0);
            this->declare_parameter("camera/right/k1", 0.0);
            this->declare_parameter("camera/right/k2", 0.0);
            this->declare_parameter("camera/right/p1", 0.0);
            this->declare_parameter("camera/right/p2", 0.0);
            this->declare_parameter("camera/right/r12", 0.0);
            this->declare_parameter("camera/right/r13", 0.0);
            this->declare_parameter("camera/right/r21", 0.0);
            this->declare_parameter("camera/right/r23", 0.0);
            this->declare_parameter("camera/right/r31", 0.0);
            this->declare_parameter("camera/right/r32", 0.0);
            this->declare_parameter("camera/right/fx_p", 0.0);
            this->declare_parameter("camera/right/fy_p", 0.0);
            this->declare_parameter("camera/right/cx_p", 0.0);
            this->declare_parameter("camera/right/cy_p", 0.0);
            this->declare_parameter("camera/right/Tx", 0.0);

            width_ = this->get_parameter("camera/left/width").as_int();
            height_ = this->get_parameter("camera/left/height").as_int();

            fx_left_ = this->get_parameter("camera/left/fx").as_double();
            fy_left_ = this->get_parameter("camera/left/fy").as_double();
            cx_left_ = this->get_parameter("camera/left/cx").as_double();
            cy_left_ = this->get_parameter("camera/left/cy").as_double();
            k1_left_ = this->get_parameter("camera/left/k1").as_double();
            k2_left_ = this->get_parameter("camera/left/k2").as_double();
            p1_left_ = this->get_parameter("camera/left/p1").as_double();
            p2_left_ = this->get_parameter("camera/left/p2").as_double();
            r12_left_ = this->get_parameter("camera/left/r12").as_double();
            r13_left_ = this->get_parameter("camera/left/r13").as_double();
            r21_left_ = this->get_parameter("camera/left/r21").as_double();
            r23_left_ = this->get_parameter("camera/left/r23").as_double();
            r31_left_ = this->get_parameter("camera/left/r31").as_double();
            r32_left_ = this->get_parameter("camera/left/r32").as_double();
            fx_p_left_ = this->get_parameter("camera/left/fx_p").as_double();
            fy_p_left_ = this->get_parameter("camera/left/fy_p").as_double();
            cx_p_left_ = this->get_parameter("camera/left/cx_p").as_double();
            cy_p_left_ = this->get_parameter("camera/left/cy_p").as_double();

            fx_right_ = this->get_parameter("camera/right/fx").as_double();
            fy_right_ = this->get_parameter("camera/right/fy").as_double();
            cx_right_ = this->get_parameter("camera/right/cx").as_double();
            cy_right_ = this->get_parameter("camera/right/cy").as_double();
            k1_right_ = this->get_parameter("camera/right/k1").as_double();
            k2_right_ = this->get_parameter("camera/right/k2").as_double();
            p1_right_ = this->get_parameter("camera/right/p1").as_double();
            p2_right_ = this->get_parameter("camera/right/p2").as_double();
            r12_right_ = this->get_parameter("camera/right/r12").as_double();
            r13_right_ = this->get_parameter("camera/right/r13").as_double();
            r21_right_ = this->get_parameter("camera/right/r21").as_double();
            r23_right_ = this->get_parameter("camera/right/r23").as_double();
            r31_right_ = this->get_parameter("camera/right/r31").as_double();
            r32_right_ = this->get_parameter("camera/right/r32").as_double();
            fx_p_right_ = this->get_parameter("camera/right/fx_p").as_double();
            fy_p_right_ = this->get_parameter("camera/right/fy_p").as_double();
            cx_p_right_ = this->get_parameter("camera/right/cx_p").as_double();
            cy_p_right_ = this->get_parameter("camera/right/cy_p").as_double();
            Tx_right_ = this->get_parameter("camera/right/Tx").as_double();

            left_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/left/camera_info",2);
            right_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/right/camera_info",2);
            timer_ = this->create_wall_timer(
                50ms, std::bind(&CameraCalibrationPub::publish_camera_info, this));
        }

    private:
        void publish_camera_info()
        {
            auto left_message = sensor_msgs::msg::CameraInfo();
            auto right_message = sensor_msgs::msg::CameraInfo();
            left_message.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            right_message.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            // Distortion matrix
            left_message.d.resize(5);
            left_message.d[0] = k1_left_;
            left_message.d[1] = k2_left_;
            left_message.d[2] = p1_left_;
            left_message.d[3] = p2_left_;
            left_message.d[4] = 0.0;

            // Intrinsic matrix
            left_message.k.fill(0.0);
            left_message.k[0] = fx_left_;
            left_message.k[2] = cx_left_;
            left_message.k[4] = fy_left_;
            left_message.k[5] = cy_left_;
            left_message.k[8] = 1.0;

            // Rectification matrix
            left_message.r.fill(0.0);

            for (size_t i = 0; i < 3; i++) {
                // Identity
                left_message.r[i + i * 3] = 1;
            }

            left_message.r[1] = r12_left_;
            left_message.r[2] = r13_left_;
            left_message.r[3] = r21_left_;
            left_message.r[5] = r23_left_;
            left_message.r[6] = r31_left_;
            left_message.r[7] = r32_left_;

            // Projection matrix
            left_message.p.fill(0.0);
            left_message.p[0] = fx_p_left_;
            left_message.p[2] = cx_p_left_;
            left_message.p[5] = fy_p_left_;
            left_message.p[6] = cy_p_left_;
            left_message.p[10] = 1.0;
            
            left_message.width = width_;
            left_message.height = height_;

            left_message.header.frame_id = "camera_link";

            // Distortion matrix
            right_message.d.resize(5);
            right_message.d[0] = k1_right_;
            right_message.d[1] = k2_right_;
            right_message.d[2] = p1_right_;
            right_message.d[3] = p2_right_;
            right_message.d[4] = 0.0;

            // Intrinsic matrix
            right_message.k.fill(0.0);
            right_message.k[0] = fx_right_;
            right_message.k[2] = cx_right_;
            right_message.k[4] = fy_right_;
            right_message.k[5] = cy_right_;
            right_message.k[8] = 1.0;

            // Rectification matrix
            right_message.r.fill(0.0);

            for (size_t i = 0; i < 3; i++) {
                // Identity
                right_message.r[i + i * 3] = 1;
            }

            right_message.r[1] = r12_right_;
            right_message.r[2] = r13_right_;
            right_message.r[3] = r21_right_;
            right_message.r[5] = r23_right_;
            right_message.r[6] = r31_right_;
            right_message.r[7] = r32_right_;

            // Projection matrix
            right_message.p.fill(0.0);
            right_message.p[0] = fx_p_right_;
            right_message.p[2] = cx_p_right_;
            right_message.p[5] = fy_p_right_;
            right_message.p[6] = cy_p_right_;
            right_message.p[3] = Tx_right_;
            right_message.p[10] = 1.0;
            
            right_message.width = width_;
            right_message.height = height_;

            right_message.header.frame_id = "camera_link_right";

            left_publisher->publish(left_message);
            right_publisher->publish(right_message);
        }
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_publisher;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_publisher;

        double fx_left_;
        double fy_left_;
        double cx_left_;
        double cy_left_;

        double k1_left_;
        double k2_left_;
        double p1_left_;
        double p2_left_;

        double r12_left_;
        double r13_left_;
        double r21_left_;
        double r23_left_;
        double r31_left_;
        double r32_left_;

        double fx_p_left_;
        double cx_p_left_;
        double fy_p_left_;
        double cy_p_left_;

        double fx_right_;
        double fy_right_;
        double cx_right_;
        double cy_right_;

        double k1_right_;
        double k2_right_;
        double p1_right_;
        double p2_right_;

        double r12_right_;
        double r13_right_;
        double r21_right_;
        double r23_right_;
        double r31_right_;
        double r32_right_;

        double fx_p_right_;
        double cx_p_right_;
        double fy_p_right_;
        double cy_p_right_;
        double Tx_right_;

        int width_;
        int height_;

        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraCalibrationPub>());
    rclcpp::shutdown();
    return 0;
}