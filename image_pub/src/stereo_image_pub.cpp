#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <stdexcept>

#include "opencv2/opencv.hpp"
#include <opencv4/opencv2/core/utility.hpp>
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/calib3d/calib3d.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/core/core_c.h"
#include "opencv4/opencv2/core/types_c.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;
using namespace cv;
using std::cout; using std::cerr; using std::endl; using std::runtime_error;

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

class StereoImagePublisher : public rclcpp::Node
{
    public:
        StereoImagePublisher()
        : Node("stereo_image_publisher")
        {
            this->declare_parameter("calibration_dir", "");
            this->declare_parameter("framerate", 0);
            this->declare_parameter("namespace", "");
            this->declare_parameter("frame_id", "");

            this->declare_parameter("camera/width", 0);
            this->declare_parameter("camera/height", 0);

            std::string calibration_dir = this->get_parameter("calibration_dir").as_string();
            intrinsic_filename_ = calibration_dir + "/intrinsics.yml";
            extrinsic_filename_ = calibration_dir + "/extrinsics.yml";

            if( !intrinsic_filename_.empty() )
            {
                // reading intrinsic parameters
                FileStorage fs(intrinsic_filename_, FileStorage::READ);
                if(!fs.isOpened())
                {
                    cout << "Failed to open file " << intrinsic_filename_.c_str() << endl;
                }
                
                fs["M1"] >> M1;
                fs["D1"] >> D1;
                fs["M2"] >> M2;
                fs["D2"] >> D2;

                M1 *= scale_;
                M2 *= scale_;

                fs.open(extrinsic_filename_, FileStorage::READ);
                if(!fs.isOpened())
                {
                    cout << "Failed to open file " << extrinsic_filename_.c_str() << endl;
                }
                
                fs["R"] >> R;
                fs["T"] >> T;
            }

            width_ = this->get_parameter("camera/width").as_int();
            height_ = this->get_parameter("camera/height").as_int();

            framerate_ = this->get_parameter("framerate").as_int();
            int period = (int) 1000.0 / framerate_;

            namespace_ = this->get_parameter("namespace").as_string();
            frame_id_ = this->get_parameter("frame_id").as_string();
            
            image_pub_left_ = this->create_publisher<sensor_msgs::msg::Image>(namespace_ + "/left/image_rect", 2);
            image_pub_right_ = this->create_publisher<sensor_msgs::msg::Image>(namespace_ + "/right/image_rect", 2);

            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period), std::bind(&StereoImagePublisher::timer_callback, this));
            
            std::string cmd0 = "nvarguscamerasrc sensor-id=";
            std::string cmd1 = " ! video/x-raw(memory:NVMM), width=3264, height=2464, format=(string)NV12, framerate=(fraction)";
            std::string cmd2 = "/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

            cam0_.open(cmd0 + std::to_string(0) + cmd1 + std::to_string(framerate_) + cmd2, cv::CAP_GSTREAMER);
            cam1_.open(cmd0 + std::to_string(1) + cmd1 + std::to_string(framerate_) + cmd2, cv::CAP_GSTREAMER);
        }

    private:
        void timer_callback()
        { 
            Mat frame0, frame1;
            Mat left, right;

            cam0_.read(frame0);
            cam1_.read(frame1);
            if (frame0.empty()) throw runtime_error("cannot read frame0 from camera");
            if (frame1.empty()) throw runtime_error("cannot read frame1 from camera");

            cvtColor(frame0, left, COLOR_BGR2GRAY);
            cvtColor(frame1, right, COLOR_BGR2GRAY);

            if (scale_ != 1.f)
            {
                Mat temp1, temp2;
                int method = scale_ < 1 ? INTER_AREA : INTER_CUBIC;
                resize(left, temp1, Size(), scale_, scale_, method);
                left = temp1;
                resize(right, temp2, Size(), scale_, scale_, method);
                right = temp2;
            }

            Size img_size = left.size();

            // Rectify two images with calibrated parameters
            Rect roi1, roi2;
            Mat Q;
            stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
            Mat map11, map12, map21, map22;
            initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
            initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
            Mat img1r, img2r;
            remap(left, img1r, map11, map12, INTER_LINEAR);
            remap(right, img2r, map21, map22, INTER_LINEAR);
            left = img1r;
            right = img2r;

            sensor_msgs::msg::Image::UniquePtr image_msg_left(new sensor_msgs::msg::Image());
            sensor_msgs::msg::Image::UniquePtr image_msg_right(new sensor_msgs::msg::Image());

            // Convert OpenCV Mat to ROS Image
            rclcpp::Time timestamp = this->get_clock()->now();

            image_msg_left->header.stamp = timestamp;
            image_msg_left->header.frame_id = frame_id_;
            image_msg_left->height = left.rows;
            image_msg_left->width = left.cols;
            image_msg_left->encoding = mat_type2encoding(left.type());
            image_msg_left->is_bigendian = false;
            image_msg_left->step = static_cast<sensor_msgs::msg::Image::_step_type>(left.step);
            image_msg_left->data.assign(left.datastart, left.dataend);

            image_msg_right->header.stamp = timestamp;
            image_msg_right->header.frame_id = frame_id_;
            image_msg_right->height = right.rows;
            image_msg_right->width = right.cols;
            image_msg_right->encoding = mat_type2encoding(right.type());
            image_msg_right->is_bigendian = false;
            image_msg_right->step = static_cast<sensor_msgs::msg::Image::_step_type>(right.step);
            image_msg_right->data.assign(right.datastart, right.dataend);

            image_pub_left_->publish(std::move(image_msg_left));
            image_pub_right_->publish(std::move(image_msg_right));
        }
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_left_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_right_;

        VideoCapture cam0_;
        VideoCapture cam1_;

        int framerate_;
        std::string namespace_;
        std::string frame_id_;
        std::string intrinsic_filename_;
        std::string extrinsic_filename_;

        int width_;
        int height_;
        float scale_ = 1.0f;

        Mat M1, D1, M2, D2;
        Mat R, T, R1, P1, R2, P2;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoImagePublisher>());
    rclcpp::shutdown();
    return 0;
}