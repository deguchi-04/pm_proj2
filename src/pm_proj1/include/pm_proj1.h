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

/// Global vars

/// ROS Publisher
ros::Publisher pub; // Global to be use outside main
std::string path = "../project_pm/src/pm_proj1/src/videoPlastic.mp4";
cv::RNG rng(12345);
cv::Point ball_center;
bool flag_first;
int thresh = 80;
int meanColor = 0;
int stdev = 0;
double area = 0;

#endif
