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
#include <geometry_msgs/msg/pose.hpp>
#include <fstream>

#define STREAM          RS2_STREAM_COLOR  // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT          RS2_FORMAT_RGB8   // rs2_format identifies how binary data is encoded within a frame      //
#define WIDTH           640 // 1920               // Defines the number of columns for each frame                         //
#define HEIGHT          480 // 1080               // Defines the number of lines for each frame                           //
#define FPS             30                // Defines the rate of frames per second                                //
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //

//all #define dimensions in mm
#define QR_Dimension 66

#define Spacing 17

#define Cube_Size 100


//from camera calibration
// const cv::Mat INTRINSIC = (cv::Mat_<float>(3, 3) << 1378.13429, 0, 954.241651,
//                                                    0, 1376.70507, 554.51173,
//                                                    0, 0, 1);
const cv::Mat INTRINSIC = (cv::Mat_<float>(3, 3) << 608.90339309, 0, 321.37995726,
                                                   0, 608.25294592, 244.79138807,
                                                   0, 0, 1);

// const cv::Mat DIST = (cv::Mat_<double>(1, 5) << -.00535442812, -0.862563774, -0.000440522884, -0.00402199855, -3.24408199);
const cv::Mat DIST = (cv::Mat_<double>(1, 5) << -0.0166, 1.06, 0, 0, -4.06);

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

            pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/cube/pose", 10);
        }

    private:
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            try {
                cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
                // detect_and_draw_qr_codes(frame);
                detect_cube_cords(frame);

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
                    double X = (centroid.x - cx_) * Z / fx_;
                    double Y = (centroid.y - cy_) * Z / fy_;
                    RCLCPP_INFO(this->get_logger(), "QR Code 3D Position: X=%f m, Y=%f m, Z=%f m", X, Y, Z);

                    // Create the pose message
                    geometry_msgs::msg::Pose pose_msg;
                    pose_msg.position.x = X;
                    pose_msg.position.y = Y;
                    pose_msg.position.z = Z;
                    pose_msg.orientation.x = 0.0;
                    pose_msg.orientation.y = 0.0;
                    pose_msg.orientation.z = 0.0;
                    pose_msg.orientation.w = 1.0;  // No rotation, neutral orientation

                    // Publish the pose
                    pose_publisher_->publish(pose_msg);
                }
            }
        }
    }
    
    void detect_cube_cords(cv::Mat& image) {

        if (!camera_info_received_) {
            RCLCPP_WARN(this->get_logger(), "Camera info not yet received.");
            return;
        }

        cv::QRCodeDetector qrDecoder;
        std::vector<cv::Point> qrCode_pixel_cords;

        //successful detection
        if (qrDecoder.detect(image, qrCode_pixel_cords)){
            if (errorCheckBoundingBox(qrCode_pixel_cords)){//find displacement of 8 cube corners
                
                //draw qr corners after successful detection and error check
                drawPoints(image, qrCode_pixel_cords, 255, 255, 0);
                
                cv::Mat qr_corners = (cv::Mat_<float>(4, 3) << 0.f, 0.f, 0.f,
                                                        QR_Dimension, 0.f, 0.f,
                                                        QR_Dimension, QR_Dimension, 0.f,
                                                        0.f, QR_Dimension, 0.f);

                cv::Mat rvec, tvec;

                //reformat qr_pixels to fit solvePnP
                cv::Mat pixelsMat(qrCode_pixel_cords.size(), 2, CV_32F);
                for (size_t i = 0; i < qrCode_pixel_cords.size(); ++i) {
                    pixelsMat.at<float>(i, 0) = static_cast<float>(qrCode_pixel_cords[i].x);
                    pixelsMat.at<float>(i, 1) = static_cast<float>(qrCode_pixel_cords[i].y);
                }

                //if solvePnP worked, never really fails
                if (cv::solvePnP(qr_corners, pixelsMat, INTRINSIC, DIST, rvec, tvec)){
                //end step 1
                //*************************************************************************************************************************************
                
                //step 2 find pixel coordinates of cube
                //*************************************************************************************************************************************
                    cv::Mat rmat;
                    cv::Rodrigues(rvec, rmat); //turn 3x1 rotation vector into 3x3 rotation matrix, units are in radians

                    //homogenous world coords of cube corners relative to the qr code
                        //format per coord: x, y, z, 1
                    cv::Mat cube_cords = (cv::Mat_<float>(8, 4) << -Spacing, -Spacing, 0.f, 1.f,
                                                                    QR_Dimension + Spacing, -Spacing, 0.f, 1.f,
                                                                    QR_Dimension + Spacing, QR_Dimension + Spacing, 0.f, 1.f,
                                                                    -Spacing, QR_Dimension + Spacing, 0.f, 1.f,
                                                                    -Spacing, -Spacing, Cube_Size, 1.f,
                                                                    QR_Dimension + Spacing, -Spacing, Cube_Size, 1.f,
                                                                    QR_Dimension + Spacing, QR_Dimension + Spacing, Cube_Size, 1.f,
                                                                    -Spacing, QR_Dimension + Spacing, Cube_Size, 1.f);

                    //essential matrix
                    //essential matrix is 4x4 matrix
                    //top left 3x3 is rotation matrix
                    //3x1 column on the right side is tvec
                    //bottom row is 0,0,0,1
                    cv::Mat essential = cv::Mat::eye(3,4, CV_32F);
                    rmat.copyTo(essential(cv::Rect(0,0, rmat.cols, rmat.rows)));
                    tvec.copyTo(essential(cv::Rect(3,0,1,tvec.rows)));

                    //camera Matrix, dot product of Intrinsic and essential
                    cv::Mat cameraMatrix = INTRINSIC * essential;

                    //get pixel coords of cube corners
                    cv::Mat cube_pixels = cv::Mat::zeros(8,2, CV_32F);

                    for (int i = 0; i < cube_cords.rows; i++){
                        cv::Mat pixel = cameraMatrix * cube_cords.row(i).t();

                        cube_pixels.at<float>(i, 0) = pixel.at<float>(0) / pixel.at<float>(2);
                        cube_pixels.at<float>(i, 1) = pixel.at<float>(1) / pixel.at<float>(2);
                    }

                    drawBoundingBox(image, cube_pixels, 0, 255, 255);

                    //end step 2
                    //*************************************************************************************************************************************

                    //step 3 get rotation and translation relative to closest face of cube
                    //*************************************************************************************************************************************
                    
                    //start with coordinates of cube with top left of front most face as world origin
                    cv::Mat cube_corners_coplanar_front = (cv::Mat_<float>(4, 3) << 0.f, 0.f, 0.f,
                                                                    Cube_Size, 0.f, 0.f,
                                                                    Cube_Size, Cube_Size, 0.f,
                                                                    0.f, Cube_Size, 0.f);
                    
                    cv::Mat frontPixels = cube_pixels.rowRange(0,4); //isolate 4 front pixel coords
                    cv::Mat cube_rvec(3, 1, CV_32F, cv::Scalar(1));
                    cv::Mat cube_tvec(3, 1, CV_32F, cv::Scalar(1));
                    
                    if (cv::solvePnP(cube_corners_coplanar_front, frontPixels, INTRINSIC, DIST, cube_rvec, cube_tvec)){ //solve for rotation and translation
                        
                        //end step 3
                        //*************************************************************************************************************************************
                        
                        //step 4, find mm displacement of all 8 corners
                        //*************************************************************************************************************************************
                        cv::Mat cube_rmat(3, 3, CV_32F, cv::Scalar(1));
                        cv::Rodrigues(cube_rvec, cube_rmat); //turn 3x1 vector into 3x3 matrix
                        
                        //cube world coords
                        cv::Mat cube_corners_origin = (cv::Mat_<float>(8, 3) << 0.f, 0.f, 0.f,
                                                                    Cube_Size, 0.f, 0.f,
                                                                    Cube_Size, Cube_Size, 0.f,
                                                                    0.f, Cube_Size, 0.f,
                                                                    0.f, 0.f, Cube_Size,
                                                                    Cube_Size, 0.f, Cube_Size,
                                                                    Cube_Size, Cube_Size, Cube_Size,
                                                                    0.f, Cube_Size, Cube_Size);

                        cv::Mat cube_xyz = cv::Mat::zeros(8, 3, CV_32F);

                        cv::Mat invIntrinsic;
                        cv::invert(INTRINSIC, invIntrinsic); //invert intrinsic matrix

                        for (int i = 0; i < cube_corners_origin.rows; i++){ //for each of the 8 coordinates
                            cv::Mat row_cord = cube_corners_origin.row(i);
                            cv::Mat column_cord = row_cord.reshape(0, 3); //reshape coordinate to 3x1

                            cv::Mat world_pixel = cv::Mat::zeros(3,1, CV_32F);
                            
                            //manual dot product of rmat and column_cord
                            for (int a = 0; a < cube_rmat.rows; a++){
                              for (int b = 0; b < column_cord.cols; b++){
                                  for (int c = 0; c < cube_rmat.cols; c++){
                                      world_pixel.at<float>(a, b) += cube_rmat.at<float>(a, c) * column_cord.at<float>(c, b);
                                    }
                                }
                            }

                            //manual addtion of world pixel and cube translation vector, both 3x1
                            for (int z = 0; z < cube_tvec.rows; z++){
                                world_pixel.at<float>(z, 0) += cube_tvec.at<float>(z, 0);
                            }

                            //assign world pixel to correct spot in cube_xyz variable
                            cube_xyz.at<float>(i, 0) = world_pixel.at<float>(0, 0);
                            cube_xyz.at<float>(i, 1) = world_pixel.at<float>(1, 0);
                            cube_xyz.at<float>(i, 2) = world_pixel.at<float>(2, 0);

                        }
                    
                        float center[3] = {0, 0, 0};
                        for (int i = 0; i<cube_xyz.rows; i++) {
                            center[0] += cube_xyz.at<float>(i, 0);
                            center[1] += cube_xyz.at<float>(i, 1);
                            center[2] += cube_xyz.at<float>(i, 2);
                        }

                        double scale_ = 0.001; // mm to meters
                        for (int i = 0; i<3; i++) center[i] = (center[i] / cube_xyz.rows) * scale_;
                        
                        RCLCPP_INFO(this->get_logger(), "QR Code 3D Position: X=%f m, Y=%f m, Z=%f m", center[0], center[1], center[2]);

                        // Create the pose message
                        geometry_msgs::msg::Pose pose_msg;
                        pose_msg.position.x = center[0];
                        pose_msg.position.y = center[0];
                        pose_msg.position.z = center[0];
                        pose_msg.orientation.x = cube_rvec.at<float>(0,0);
                        pose_msg.orientation.y = cube_rvec.at<float>(1,0);
                        pose_msg.orientation.z = cube_rvec.at<float>(2,0);
                        pose_msg.orientation.w = 1.0;  // No rotation, neutral orientation

                        // Publish the pose
                        pose_publisher_->publish(pose_msg);
                    }                                               
                }
            }
        }
    }
    bool qrCodeDetect(int height, int width, cv::Mat wind, std::vector<cv::Point> &pixels){
        //int height = y value for top left of the window portion of the main image, i value from nested for loop in main
        //int width = x value for top left of the window portion of the main image, j value from nested for loop in main
        //cv::Mat wind, window matrix
        //std::vector<cv::Point> &pixels, pixel vector passed by reference that will contain the 4 pixel coordinates of the qr code after successful detection
        
        //qr code detector object
        cv::QRCodeDetector qr = cv::QRCodeDetector();

        //detect returns true or false
        if (qr.detect(wind, pixels)){

            //add i, j to pixels, the current pixels are relative to the window not the image
            for (int i = 0; i < 4; i++){
                pixels[i].x = pixels[i].x + width;
                pixels[i].y = pixels[i].y + height;
            }
            return true;
        }
        return false;
    };

    bool errorCheckBoundingBox(const std::vector<cv::Point>& pixels) {
        
        // Assign variables
        int top_left_x = pixels[0].x;
        int top_left_y = pixels[0].y;
        int top_right_x = pixels[1].x;
        int top_right_y = pixels[1].y;
        int bottom_right_x = pixels[2].x;
        int bottom_right_y = pixels[2].y;
        int bottom_left_x = pixels[3].x;
        int bottom_left_y = pixels[3].y;

        // Check if distances are acceptable
        //**************************************************************************************************************************************
        //the final check is to see if the sides of the bounding quadrangle are acceptable
        //a false positive detection can pass the parallel test and still have a non square bounding box
            //we look to see if the top side and right side length are within 10% of each other, no need to check all four sides 
        double top_dist_squared = (top_left_x - top_right_x) * (top_left_x - top_right_x) + (top_left_y - top_right_y) * (top_left_y - top_right_y);
        double right_dist_squared = (bottom_right_x - top_right_x) * (bottom_right_x - top_right_x) + (bottom_right_y - top_right_y) * (bottom_right_y - top_right_y);

        if (top_dist_squared < 0.75 * right_dist_squared || top_dist_squared > 1.3 * right_dist_squared) {
            return false; // Top dist is less than 90% or greater than 110% of right distance
        }
        //**************************************************************************************************************************************

        // Check if lines are parallel
        //**************************************************************************************************************************************
        //this check is done to eliminate incorrect detections of qr corners 
        //the qr code detector sometimes "finds" the qr corners because it finds four points in the image that match the gradient 
            //of the qr corners
            //this check eliminates lots of those false positives by checking the top/bottom and left/ride sides of the bounding quadrangle
            //are within 5 degrees of parallel
        // Get vectors
        int top_vector_x = top_right_x - top_left_x;
        int top_vector_y = top_right_y - top_left_y;
        int right_vector_x = bottom_right_x - top_right_x;
        int right_vector_y = bottom_right_y - top_right_y;
        int bottom_vector_x = bottom_right_x - bottom_left_x;
        int bottom_vector_y = bottom_right_y - bottom_left_y;
        int left_vector_x = bottom_left_x - top_left_x;
        int left_vector_y = bottom_left_y - top_left_y;

        // Calculate dot products and vector magnitudes
        float dot_top_bottom = top_vector_x * bottom_vector_x + top_vector_y * bottom_vector_y;
        float dot_left_right = left_vector_x * right_vector_x + left_vector_y * right_vector_y;
        float mag_top = sqrt(top_vector_x * top_vector_x + top_vector_y * top_vector_y);
        float mag_bottom = sqrt(bottom_vector_x * bottom_vector_x + bottom_vector_y * bottom_vector_y);
        float mag_left = sqrt(left_vector_x * left_vector_x + left_vector_y * left_vector_y);
        float mag_right = sqrt(right_vector_x * right_vector_x + right_vector_y * right_vector_y);

        // Calculate angles (in radians)
        float angle_top_bottom_pair = acos(dot_top_bottom / (mag_top * mag_bottom));
        float angle_left_right_pair = acos(dot_left_right / (mag_left * mag_right));

        // Convert angles to degrees
        const float radians_to_degrees = 180.0 / 3.1415;
        angle_top_bottom_pair *= radians_to_degrees;
        angle_left_right_pair *= radians_to_degrees;

        // Check tolerance
        if (angle_left_right_pair > 10 || angle_top_bottom_pair > 10) {
            return false;
        }
        //**************************************************************************************************************************************

        // Return true if passes all checks
        return true;

        //feel free to alter the order of these error checks there might be some performance gain in eithe rruntime or accuracy by fine tuning the parameteres
    }
    
    void drawBoundingBox(cv::Mat &img, cv::Mat pixels, uint8_t r, uint8_t g, uint8_t b){
        //visual tool for showing output of cube pixels
        //gives good indecation of the accuracy of mm measurements

        //draw bounding box based on pixel coordinates
        for (int i = 0; i < pixels.rows; ++i) {
            int x = static_cast<int>(pixels.at<float>(i, 0));
            int y = static_cast<int>(pixels.at<float>(i, 1));
            // Draw a circle on the image for each point
            cv::circle(img, cv::Point(x, y), 10, cv::Scalar(r, g, b), -1); // -1 indicates fill the circle with color
        }
    }

    void drawPoints(cv::Mat &img, std::vector<cv::Point> pixels, uint8_t r, uint8_t g, uint8_t b){
        //draws circles over the corners of the detected qr code
        for (const auto& point : pixels) {
            int x = point.x;
            int y = point.y;
            // Draw a circle on the image for each point
            cv::circle(img, point, 10, cv::Scalar(r, g, b), -1); // -1 indicates fill the circle with color
        }
    }

    image_transport::Subscriber color_subscriber_;
    image_transport::Subscriber depth_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    image_transport::Publisher image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    cv::Mat current_frame_;
    cv::Mat current_depth_frame_;
    float scale_ = 0.001; // meters
    bool camera_info_received_ = false;
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QRCodeDetectorNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}