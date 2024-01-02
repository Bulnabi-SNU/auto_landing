#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/distortion_models.hpp"

using namespace std::chrono_literals;
using namespace cv;

std::string
mat_type2encoding(int mat_type)
{
    switch (mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
    }
}

class MonoImagePublisher : public rclcpp::Node
{
    public:
        MonoImagePublisher()
        : Node("mono_image_publisher")
        {
            this->declare_parameter("framerate", 30);
            this->declare_parameter("sensor_id", 1);
            this->declare_parameter("namespace", "/camera");
            this->declare_parameter("frame_id", "camera_link");

            this->declare_parameter("camera/width", 640);
            this->declare_parameter("camera/height", 480);

            this->declare_parameter("camera/fx", 417.849766);
            this->declare_parameter("camera/fy", 557.010356);
            this->declare_parameter("camera/cx", 316.875446);
            this->declare_parameter("camera/cy", 235.603508);
            this->declare_parameter("camera/k1", 0.009927);
            this->declare_parameter("camera/k2", -0.056084);
            this->declare_parameter("camera/p1", -0.000699);
            this->declare_parameter("camera/p2", -0.000647);
            this->declare_parameter("camera/r12", 0.0);
            this->declare_parameter("camera/r13", 0.0);
            this->declare_parameter("camera/r21", 0.0);
            this->declare_parameter("camera/r23", 0.0);
            this->declare_parameter("camera/r31", 0.0);
            this->declare_parameter("camera/r32", 0.0);
            this->declare_parameter("camera/fx_p", 406.141862);
            this->declare_parameter("camera/fy_p", 542.973528);
            this->declare_parameter("camera/cx_p", 315.419608);
            this->declare_parameter("camera/cy_p", 234.546206);
            this->declare_parameter("camera/Tx", 0.0);
            framerate_ = 30;
            int period = (int) 1000.0 / framerate_;

            int sensor_id = 0;

            std::string namespace_ = "/camera" ;
            frame_id_ = this->get_parameter("frame_id").as_string();
            
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(namespace_ + "/image_raw", 2);
            image_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(namespace_ + "/camera_info", 2);

            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period), std::bind(&MonoImagePublisher::timer_callback, this));
            
            std::string cmd0 = "nvarguscamerasrc sensor-id=";
            std::string cmd1 = " ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=(fraction)";
            std::string cmd2 = "/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1920, height=1080, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

            cam_.open(cmd0 + std::to_string(sensor_id) + cmd1 + std::to_string(framerate_) + cmd2, cv::CAP_GSTREAMER);
        }

    private:
        void timer_callback()
        { 
            Mat frame;
            cam_.read(frame);

            sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

            // Convert OpenCV Mat to ROS Image
            rclcpp::Time timestamp = this->get_clock()->now();
            image_msg->header.stamp = timestamp;
            image_msg->header.frame_id = frame_id_;
            image_msg->height = frame.rows;
            image_msg->width = frame.cols;
            image_msg->encoding = mat_type2encoding(frame.type());
            image_msg->is_bigendian = false;
            image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            image_msg->data.assign(frame.datastart, frame.dataend);

            auto cam_info_msg = sensor_msgs::msg::CameraInfo();
            cam_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            // Distortion matrix
            cam_info_msg.d.resize(5);
            cam_info_msg.d[0] = 0.018063;
            cam_info_msg.d[1] = -0.063093;
            cam_info_msg.d[2] = -0.000450;
            cam_info_msg.d[3] = 0.001720;
            cam_info_msg.d[4] = 0.0;

            // Intrinsic matrix
            cam_info_msg.k.fill(0.0);
            cam_info_msg.k[0] = 1263.504838;
            cam_info_msg.k[2] = 982.091350;
            cam_info_msg.k[4] = 1263.259169;
            cam_info_msg.k[5] = 548.312618;
            cam_info_msg.k[8] = 1.0;

            // Rectification matrix
            cam_info_msg.r.fill(0.0);

            for (size_t i = 0; i < 3; i++) {
                // Identity
                cam_info_msg.r[i + i * 3] = 1;
            }

            cam_info_msg.r[1] = 0.0;
            cam_info_msg.r[2] = 0.0;
            cam_info_msg.r[3] = 0.0;
            cam_info_msg.r[5] = 0.0;
            cam_info_msg.r[6] = 0.0;
            cam_info_msg.r[7] = 0.0;

            // Projection matrix
            cam_info_msg.p.fill(0.0);
            cam_info_msg.p[0] = 1248.006104;
            cam_info_msg.p[2] = 987.309636;
            cam_info_msg.p[5] = 1264.111328;
            cam_info_msg.p[6] = 547.503061;
            cam_info_msg.p[3] = 0.0;
            cam_info_msg.p[10] = 1.0;
            
            cam_info_msg.width = 1920;
            cam_info_msg.height = 1080;

            cam_info_msg.header.frame_id = frame_id_;
            cam_info_msg.header.stamp = timestamp;

            image_info_pub_->publish(cam_info_msg);
            image_pub_->publish(std::move(image_msg));
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr image_info_pub_;
        VideoCapture cam_;

        int framerate_;
        std::string namespace_;

        double fx_;
        double fy_;
        double cx_;
        double cy_;

        double k1_;
        double k2_;
        double p1_;
        double p2_;

        double r12_;
        double r13_;
        double r21_;
        double r23_;
        double r31_;
        double r32_;

        double fx_p_;
        double cx_p_;
        double fy_p_;
        double cy_p_;
        double Tx_;

        int width_;
        int height_;

        std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonoImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
