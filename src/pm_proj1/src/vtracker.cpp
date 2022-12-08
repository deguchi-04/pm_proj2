#include "pm_proj1.h"
namespace plt = matplotlibcpp;

geometry_msgs::Vector3 center;
sensor_msgs::ImagePtr frame;

void cbPose(const sensor_msgs::ImageConstPtr &msg_frame)
{
    try
    {
        // Received center
        cv::Mat img = cv_bridge::toCvShare(msg_frame, "bgr8")->image;

        cv::imshow("Frame", cv_bridge::toCvShare(msg_frame, "bgr8")->image);

        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("ERRO");
    }
}

void cbPose2(const geometry_msgs::Vector3 &msg)
{
    if(msg.x <0 || msg.y <0){
        
    }else{
        center.x = msg.x;
        center.y = msg.y;
        ROS_WARN_STREAM("Inside callback Obs!");
        ROS_WARN_STREAM("center=(" << msg.x << "," << msg.y << ")");
        
    }
    
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vtracker");
    ros::NodeHandle n_center;
    ros::NodeHandle n_frame;
    cv::namedWindow("Frame");
    image_transport::ImageTransport it(n_frame);
    // Create the node handles to establish the program as a ROS node
    ros::Subscriber sub = n_center.subscribe("vloarder/ball_center", 1, cbPose2);
    image_transport::Subscriber sub_frame = it.subscribe("vloarder/frame", 1, cbPose);

    ros::Publisher center_ball_publisher_tracked;
    center_ball_publisher_tracked = n_center.advertise<geometry_msgs::Vector3>("vtracker/ball_center_tracker", 1000);

    /////////////////////////EXERCISE 2////////////////////////////

    float fps = 30;

    cv::KalmanFilter kalman_fil(4, 2);

    float transitionMatrixValues[4][4] = {{1, 0, 1 / fps, 0},
                                          {0, 1, 0, 1 / fps},
                                          {0, 0, 1, 0},
                                          {0, 0, 0, 1}};
    kalman_fil.transitionMatrix = cv::Mat(4, 4, CV_32F, transitionMatrixValues);

    float measurementMatrixValues[2][4] = {{1, 0, 0, 0},
                                           {0, 1, 0, 0}};
    kalman_fil.measurementMatrix = cv::Mat(2, 4, CV_32F, measurementMatrixValues);

    float processNoiseMatrix[4][4] = {{1e-3, 0, 0, 0},
                                      {0, 1e-3, 0, 0},
                                      {0, 0, 1e-3, 0},
                                      {0, 0, 0, 1e-3}};
    kalman_fil.processNoiseCov = cv::Mat(4, 4, CV_32F, processNoiseMatrix);

    float measurementNoiseMatrix[2][2] = {{1e-4, 0},
                                          {0, 1e-4}};
    kalman_fil.measurementNoiseCov = cv::Mat(2, 2, CV_32F, measurementNoiseMatrix);

    float errorCovPostMatrix[4][4] = {{0.5, 0, 0, 0},
                                      {0, 0.5, 0, 0},
                                      {0, 0, 0.5, 0},
                                      {0, 0, 0, 0.5}};
    kalman_fil.errorCovPost = cv::Mat(4, 4, CV_32F, errorCovPostMatrix);

    cv::Mat mp(2, 1, CV_32F, cv::Scalar::all(0));
    std::vector<int> predicted_x;
    std::vector<int> predicted_y;
    std::vector<double> observation_x;
    std::vector<double> observation_y;
    cv::Mat tp;

    while (ros::ok())
    {
        if (center.x > -1)
        {
            mp.at<float>(0, 0) = (float)center.x;
            mp.at<float>(1, 0) = (float)center.y;
            kalman_fil.correct(mp);
        }
        else
        {
            mp.at<float>(0, 0) = 0;
            mp.at<float>(1, 0) = 0;
        }

        tp = kalman_fil.predict();

        predicted_x.push_back((int)tp.at<float>(0));
        predicted_y.push_back((int)tp.at<float>(1));

        observation_x.push_back(center.x);
        observation_y.push_back(center.y);

        geometry_msgs::Vector3 center_tracked;
        center_tracked.x = tp.at<float>(0);
        center_tracked.y = tp.at<float>(1);
        center_tracked.z = 0;

        center_ball_publisher_tracked.publish(center_tracked);
        
        ros::spinOnce();
    }
    

    plt::figure(1);
    plt::subplot(2,2,1);
    plt::plot(observation_x, observation_y, "g-", {{"label", "Observation"}});
    plt::subplot(2,2,2);
    plt::plot(predicted_x, predicted_y, "b*", {{"label", "Predicted"}});
    plt::legend("lower right");
    plt::xlabel("x");
    plt::ylabel("y");
    plt::show();
    // Spin - Infinite loop to ask ROS to read all pending callbacks

    cv::destroyWindow("Frame");
    // Close the file

    return 0;
}