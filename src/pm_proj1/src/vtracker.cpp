#include "pm_proj1.h"


geometry_msgs::Vector3 center;
   

void cbPose(const sensor_msgs::ImagePtr &msg_frame)
{
    if (!flag_first)
    {
        // Received center
        // FAZER direiro o subscriber
        ROS_WARN_STREAM("Inside callback!");
        ROS_WARN_STREAM("center=(" << ball_center.x << "," << ball_center.y << ")");
        flag_first = true;
    }
}

void cbPose2(const geometry_msgs::Vector3 &msg)
{
    if (!flag_first)
    {
        // Received center
        // FAZER direiro o subscriber
        center.x = msg.x;
        center.y = msg.y;
        ROS_WARN_STREAM("Inside callback!");
        ROS_WARN_STREAM("center=(" << msg.x << "," << msg.y << ")");
        flag_first = true;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vtracker");
    ros::NodeHandle n_center;
    // Create the node handles to establish the program as a ROS node
    ros::Subscriber sub = n_center.subscribe("vloarder/ball_center",1,cbPose2);


/////////////////////////EXERCISE 2////////////////////////////
    
    std::vector<float> observation_x;
    std::vector<float> observation_y;
    
    int fps = 30;
    
    float xinit = observation_x[0];
    float yinit = observation_y[0];

    float vxinit = (observation_x[1] - observation_x[0]) * fps;
    float vyinit = (observation_y[1] - observation_y[0]) * fps;

    cv::KalmanFilter kalman_fil(4,2);

    kalman_fil.statePost.at<float>(0) = (float)xinit;
    kalman_fil.statePost.at<float>(1) = (float)yinit;
    kalman_fil.statePost.at<float>(2) = (float)vxinit;
    kalman_fil.statePost.at<float>(3) = (float)vyinit;

    kalman_fil.statePre.at<float>(0) = (float)xinit;
    kalman_fil.statePre.at<float>(1) = (float)yinit;
    kalman_fil.statePre.at<float>(2) = (float)vxinit;
    kalman_fil.statePre.at<float>(3) = (float)vyinit;

    float transitionMatrixValues[4][4] = { {1, 0, 1/fps, 0}, 
                                           {0, 1, 0, 1/fps},
                                           {0, 0, 1, 0},
                                           {0, 0, 0, 1} };
    kalman_fil.transitionMatrix = cv::Mat(4, 4, CV_32F, transitionMatrixValues);

    float measurementMatrixValues[2][4] = { {1, 0, 0, 0},
                                            {0, 1, 0, 0} };
    kalman_fil.measurementMatrix = cv::Mat(2, 4, CV_32F, measurementMatrixValues);

    float processNoiseMatrix[4][4] = { {1e-3, 0, 0, 0}, 
                                       {0, 1e-3, 0, 0},
                                       {0, 0, 1e-3, 0},
                                       {0, 0, 0, 1e-3} };
    kalman_fil.processNoiseCov = cv::Mat(4, 4, CV_32F, processNoiseMatrix);

    float measurementNoiseMatrix[2][2] = { {1e-4, 0},
                                           {0, 1e-4} };
    kalman_fil.measurementNoiseCov = cv::Mat(2, 2, CV_32F, measurementNoiseMatrix);

    float errorCovPostMatrix[4][4] = { {0.5, 0, 0, 0},
                                       {0, 0.5, 0, 0},
                                       {0, 0, 0.5, 0},
                                       {0, 0, 0, 0.5} };
    kalman_fil.errorCovPost = cv::Mat(4, 4, CV_32F, errorCovPostMatrix);


    cv::Mat mp(2, 1, CV_32F, cv::Scalar::all(0));
    std::vector<int> predicted_x;
    std::vector<int> predicted_y;

    for (int i = 0; i < observation_x.size(); i++) {
        if (observation_x[i] > -1) {

            mp.at<float>(0, 0) = (float)observation_x[i];
            mp.at<float>(1, 0) = (float)observation_y[i];
            kalman_fil.correct(mp);
        }
        else {
            mp.at<float>(0, 0) = 0;
            mp.at<float>(1, 0) = 0;
        }

        cv::Mat tp = kalman_fil.predict();
        predicted_x.push_back((int)tp.at<float>(0));
        predicted_y.push_back((int)tp.at<float>(1));
    }


   // plt.figure(figsize=(16, 6));
    //plt.plot(observation_x, observation_y, linestyle='', marker='x', color='g', label='observation');
   // plt.plot(predicted_x, predicted_y, linestyle='', color='b', label='tracked');
   // plt.legend(loc='lower right');
   // plt.xlabel('x');
   // plt.ylabel('y');
   // plt.show();

    
    std::cout << " center: " << center << std::endl;
    // Spin - Infinite loop to ask ROS to read all pending callbacks
    ros::spin();
    return 0;
}