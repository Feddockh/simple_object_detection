#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>
#include <vector>
#include "simple_object_detection/qrCode.h"

// #define STREAM          RS2_STREAM_COLOR  // rs2_stream is a types of data provided by RealSense device           //
// #define FORMAT          RS2_FORMAT_RGB8   // rs2_format identifies how binary data is encoded within a frame      //
// #define WIDTH           640 // 1920               // Defines the number of columns for each frame                         //
// #define HEIGHT          480 // 1080               // Defines the number of lines for each frame                           //
// #define FPS             30                // Defines the rate of frames per second                                //
// #define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //
// #define WINDOW_WIDTH    540
// #define WINDOW_HEIGHT   540
// #define STEP_SIZE_LR    60
// #define STEP_SIZE_UD    60


class QRCodeDetectorNode : public rclcpp::Node {
    public:
        QRCodeDetectorNode() : Node("qr_code_detector_node") {}

        void initialize() {

            // Using shared_from_this() after the object is fully constructed
            auto node_shared_ptr = shared_from_this();
            image_transport::ImageTransport it(node_shared_ptr);

            color_subscriber_ = it.subscribe("/camera/camera/color/image_raw", 10, 
                &QRCodeDetectorNode::image_callback, this);

            depth_subscriber_ = it.subscribe("/camera/camera/aligned_depth_to_color/image_raw", 10,
                &QRCodeDetectorNode::depth_callback, this);

            camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/camera/aligned_depth_to_color/camera_info", 10,
                std::bind(&QRCodeDetectorNode::camera_info_callback, this, std::placeholders::_1));

            image_publisher_ = image_transport::create_publisher(this, "/qr_codes/detected_images");
        }

    private:
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            try {
                cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
                // cv::Mat frame = cv_bridge::toCvShare(msg, "rgb8")->image;
                detect_and_draw_qr_codes(frame);
                // detect_cube(frame);

                // Convert back to ROS message and publish
                auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                image_publisher_.publish(img_msg);
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }

        void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            try {
                current_depth_frame_ = cv_bridge::toCvShare(msg, msg->encoding)->image.clone();
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }

        void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
            fx_ = msg->k[0];  // Focal length x
            fy_ = msg->k[4];  // Focal length y
            cx_ = msg->k[2];  // Principal point x
            cy_ = msg->k[5];  // Principal point y
            camera_info_received_ = true;
            // RCLCPP_INFO(this->get_logger(), "Camera info received.");
        }

    void detect_and_draw_qr_codes(cv::Mat& image) {

        if (!camera_info_received_) {
            RCLCPP_WARN(this->get_logger(), "Camera info not yet received.");
            return;
        }

        cv::QRCodeDetector qrDecoder;
        std::vector<cv::Point> points;
        cv::Mat bbox;

        if (qrDecoder.detect(image, points)) {

            // Draw the bounding box around the QR code
            bbox = cv::Mat(points).reshape(1);
            cv::polylines(image, bbox, true, cv::Scalar(0, 255, 0), 2);

            // Calculate the centroid of the QR code bounding box
            if (!points.empty()) {

                cv::Point centroid(0, 0);
                for (const auto& pt : points) {
                    centroid.x += pt.x;
                    centroid.y += pt.y;
                }
                centroid.x /= points.size();
                centroid.y /= points.size();

                // Draw the center point
                cv::circle(image, centroid, 5, cv::Scalar(0, 0, 255), -1); // Red circle at the centroid

                // Display the coordinates if depth is available and x and y coordinates are valid
                if (!current_depth_frame_.empty() && centroid.x >= 0 && centroid.x < current_depth_frame_.cols && centroid.y >= 0 && centroid.y < current_depth_frame_.rows) {
                    double depth = current_depth_frame_.at<uint16_t>(centroid.y, centroid.x);
                    double Z = depth * scale_;
                    double X = (centroid.x - cx_) * Z / fx_ * scale_;
                    double Y = (centroid.y - cy_) * Z / fy_ * scale_;
                    RCLCPP_INFO(this->get_logger(), "QR Code 3D Position: X=%f m, Y=%f m, Z=%f m", X, Y, Z);
                }
            }
        }
    }

    image_transport::Subscriber color_subscriber_;
    image_transport::Subscriber depth_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    image_transport::Publisher image_publisher_;
    cv::Mat current_frame_;
    cv::Mat current_depth_frame_;
    float scale_ = 0.001; // meters
    bool camera_info_received_ = false;
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
};


// void detect_cube(cv::Mat& image) {

//     RCLCPP_INFO(this->get_logger(), "Image dimensions: %dx%d", image.cols, image.rows);

//     // Check that the defined constants match the image dimensions
//     if (image.cols < WIDTH || image.rows < HEIGHT) {
//         RCLCPP_ERROR(this->get_logger(), "Defined dimensions exceed image size.");
//         return;  // Early return to prevent processing with incorrect dimensions
//     }

//     std::vector<cv::Point> qrCode_pixel_cords;

//     //window search
//     cv::Mat window(WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC3);

//     bool break_loop = false;
    
//     for (int i = 0; i <= HEIGHT - WINDOW_HEIGHT; i += STEP_SIZE_UD){
//         for (int j = 0; j <= WIDTH - WINDOW_WIDTH; j += STEP_SIZE_LR) {

//             //get sliding window
//             cv::Rect roi(j, i, WINDOW_WIDTH, WINDOW_HEIGHT);

//             // Check if ROI exceeds image bounds
//             if (j + WINDOW_WIDTH > image.cols || i + WINDOW_HEIGHT > image.rows) {
//                 RCLCPP_WARN(this->get_logger(), "ROI exceeds image bounds. Skipping.");
//                 continue;  // Skip this iteration
//             }

//             // Extract the window
//             window = image(roi);
            
//             if (qrCodeDetect(i, j, window, qrCode_pixel_cords)){ //detect qr code
//                 RCLCPP_INFO(this->get_logger(), "here1");
//                 // if (errorCheckBoundingBox(qrCode_pixel_cords, j, i, WINDOW_WIDTH, WINDOW_HEIGHT)){ //error check qr detection
//                 //     std::cout<<"found"<<std::endl;
//                 //     break_loop = !break_loop;
//                 //     cubeData cube = getBoundingCube(window, qrCode_pixel_cords);
                    
//                 //     //visual debugging tools
//                 //     // drawBoundingBox(rgb_image, cube.pixel);
//                 //     // drawPoints(rgb_image, qrCode_pixel_cords);
//                 //     // RCLCPP_INFO(this->get_logger(), cube.xyz);
//                 //     std::cout<<cube.xyz<<std::endl;
//                 //     // cv::imshow("found in window", window);
//                 //     // cv::waitKey(5000);
//                 //     // cv::destroyAllWindows();
//                 // }
//                 //std::cout<<"failed error check"<<std::endl;
//             }

//             if (break_loop){
//                 break;
//             }
//         }
//         if (break_loop){
//             break;
//         }
//     }
// }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QRCodeDetectorNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
