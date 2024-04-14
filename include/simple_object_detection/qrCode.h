#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <math.h>
#include <time.h>

//struct for returning all data about found cube
struct cubeData{
    //pixel coordinates of 8 points
    cv::Mat pixel;

    //real world locations of 8 points, in mm
    cv::Mat xyz;

    //rotation matrix
    cv::Mat rmat;

    //translation vector
    cv::Mat tvec;
};

//all #define dimensions in mm
#define QR_Dimension 66

#define Spacing 17

#define Cube_Size 100

//from camera calibration
const cv::Mat INTRINSIC = (cv::Mat_<float>(3, 3) << 1378.13429, 0, 954.241651,
                                                   0, 1376.70507, 554.51173,
                                                   0, 0, 1);

// const cv::Mat DIST = (cv::Mat_<double>(1, 5) << -.00535442812, -0.862563774, -0.000440522884, -0.00402199855, -3.24408199);
const cv::Mat DIST = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);


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

//sped up version
//I know this looks a bit long, but this is an optimized version of a function I had already written
//it does decrease runtime by at least 1 second per full image
bool errorCheckBoundingBox(const std::vector<cv::Point>& pixels, int w, int h, int window_width, int window_height, int buffer_size = 20) {
    
    // Assign variables
    int top_left_x = pixels[0].x;
    int top_left_y = pixels[0].y;
    int top_right_x = pixels[1].x;
    int top_right_y = pixels[1].y;
    int bottom_right_x = pixels[2].x;
    int bottom_right_y = pixels[2].y;
    int bottom_left_x = pixels[3].x;
    int bottom_left_y = pixels[3].y;

    // Precompute values

    // Check if pixels too close to bounding box edge
    //**************************************************************************************************************************************
    //we do this check to make sure that the pixel coordinates are somewhat centered within the window
        //if they are not centered this has knock on effects on the accuracy of the milimeter measurements of the 8 corners of the cube
    
    int w_buffer = w + buffer_size ;
    int w_width_buffer = w + window_width - buffer_size;
    int h_buffer = h + buffer_size;
    int h_height_buffer = h + window_height - buffer_size;
    for (int i = 0; i < 4; i++) {
        // Check pixel x dimension
        if (pixels[i].x < w_buffer || pixels[i].x > w_width_buffer) {
            return false;
        }

        // Check pixel y dimension
        if (pixels[i].y < h_buffer || pixels[i].y > h_height_buffer) {
            return false;
        }
    }
    //**************************************************************************************************************************************
    

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


void drawBoundingBox(cv::Mat &img, cv::Mat pixels){
    //visual tool for showing output of cube pixels
    //gives good indecation of the accuracy of mm measurements

    //draw bounding box based on pixel coordinates
    for (int i = 0; i < pixels.rows; ++i) {
        int x = static_cast<int>(pixels.at<float>(i, 0));
        int y = static_cast<int>(pixels.at<float>(i, 1));
        // Draw a circle on the image for each point
        cv::circle(img, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), -1); // -1 indicates fill the circle with color
    }
}

void drawPoints(cv::Mat &img, std::vector<cv::Point> pixels){
    //draws circles over the corners of the detected qr code
    for (const auto& point : pixels) {
        int x = point.x;
        int y = point.y;
        // Draw a circle on the image for each point
        cv::circle(img, point, 10, cv::Scalar(0, 255, 0), -1); // -1 indicates fill the circle with color
    }
}

cubeData getBoundingCube(cv::Mat &img, std::vector<cv::Point> qr_pixels){
    //this function finds the pixel coordinates, rotation, translation, and mm measurements of the cube corners from the camera center
    //all of this is returned to main in a cubeData struct

    //this function goes through the following steps
    //1) find rotation and translation of the qr code
    //2) find pixel coordinates of 8 cube corners
    //3) use pixel coordinates of 4 cube corners on the front face to solve for cube rotation and translation
    //4) use the cube rotation and translation in combination with cube world coordinates to solve for mm displacement

    cubeData returnval;

    //step 1 find rotation and translation of qr code
    //*************************************************************************************************************************************
    //real world coordinates of qr code
    //set the origin to be the top left corner, points go in clockwise order around the square
    cv::Mat qr_corners = (cv::Mat_<float>(4, 3) << 0.f, 0.f, 0.f,
                                                        QR_Dimension, 0.f, 0.f,
                                                        QR_Dimension, QR_Dimension, 0.f,
                                                        0.f, QR_Dimension, 0.f);

    cv::Mat rvec, tvec;

    //reformat qr_pixels to fit solvePnP
    cv::Mat pixelsMat(qr_pixels.size(), 2, CV_32F);
    for (size_t i = 0; i < qr_pixels.size(); ++i) {
        pixelsMat.at<float>(i, 0) = static_cast<float>(qr_pixels[i].x);
        pixelsMat.at<float>(i, 1) = static_cast<float>(qr_pixels[i].y);
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

                cv::Mat transposed_rmat = cube_rmat.t(); //transpose rmat

                cv::Mat world_pixel = cv::Mat::zeros(3,1, CV_32F);
                
                //manual dot product of transposed rmat and column_cord
                for (int a = 0; a < transposed_rmat.rows; a++){
                    for (int b = 0; b < column_cord.cols; b++){
                        for (int c = 0; c < transposed_rmat.cols; c++){
                            world_pixel.at<float>(a, b) += transposed_rmat.at<float>(a, c) * column_cord.at<float>(c, b);
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

            // assemble return struct
            returnval.pixel = cube_pixels;
            returnval.xyz = cube_xyz;
            returnval.rmat = cube_rmat;
            returnval.tvec = cube_tvec;
        }                                               
    }
    return returnval;
};
