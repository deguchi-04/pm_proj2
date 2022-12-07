#ifndef PM_PROJ1_H
#define PM_PROJ1_H

#include <iostream>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> //ROS msg that will be published to vtracker
#include <turtlesim/Pose.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include<matplotlibcpp.h>
/// Global vars

// name of the windows
const cv::String window_capture_name = "Video Capture";
const cv::String window_detection_name = "Object Detection";
const cv::String window_cont_name = "Contours Detection";
std::string name, name2;
std::string path = "../project_pm/src/pm_proj1/src/videoPlastic.mp4";
cv::RNG rng(12345);
cv::Point ball_center;
cv::Mat original_frame;
bool flag_first;
int thresh = 80;
int meanColor = 0;
int stdev = 0;
double area = 0;

int lowH = 0;
int lowS = 0;
int lowV = 0;
int highH = 180;
int highS = 255;
int highV = 255;

std::vector<double> vec;
#endif
