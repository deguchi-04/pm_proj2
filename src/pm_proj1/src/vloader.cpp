#include "pm_proj1.h"

using namespace std;
using namespace cv;

const int max_value_H = 360/2;
const int max_value = 255;

const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
const String window_cont_name = "Contours Detection";

int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}



/////////////////////////EXERCISE 1/////////////////////////////////////
void method1(string path, int mean, int std, int x, int y){
    string name;
    if(path == "../project_pm/src/pm_proj1/src/videoPlastic.mp4"){
        name = "p1_plastic.mp4";
    }
    else if(path == "../project_pm/src/pm_proj1/src/videoTennis.mp4"){
        name = "p1_tennis.mp4";
    }
    // Open video
    cv::VideoCapture cap(path);

    // Check if video opened successfully
    if(!cap.isOpened()){
        std::cout << "Error opening video stream or file" << std::endl;
        return;
    }

    namedWindow(window_capture_name, WINDOW_NORMAL);
    namedWindow(window_detection_name, WINDOW_NORMAL);
    namedWindow(window_cont_name, WINDOW_NORMAL);
    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    Mat frame, frame_HSV, frame_threshold, cont;
    int fps = 30;

    // Acquire input size
    cv::Size frame_size = cv::Size((int) cap.get(cv::CAP_PROP_FRAME_WIDTH), (int) cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    cv::VideoWriter writer(name, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), fps, frame_size);

    cv::Scalar lowerBound = cv::Scalar(30, 40, 0);
    cv::Scalar upperBound = cv::Scalar(180, 255,255);

    cv::Scalar color = cv::Scalar(0, 0, 50);

    // Iterate through frames
    while(cap.isOpened()){

        ///////////////////Alinea A//////////////////////
        // Capture next frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
        {

            break;
        }
        

        
        ////////////////////Alinea B//////////////////////
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        
        ///////////////////// Alinea D //////////////////////////////
        cv::Moments m = moments(frame_threshold, false);
        cv::Point com(m.m10 / m.m00, m.m01 / m.m00);
        ball_center = com;
        cv::drawMarker(frame, com, color, cv::MARKER_TILTED_CROSS, 10, 5);
        cv::drawMarker(frame_threshold, com, color, cv::MARKER_TILTED_CROSS, 10, 5);
        
        
        Canny( frame_threshold, cont, thresh, thresh*2 );
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours( cont, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
        Mat drawing = Mat::zeros( cont.size(), CV_8UC3 );
        for( size_t i = 0; i< contours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            drawContours( frame, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
            //////////////// Alinea E///////////////////
            area=contourArea(contours[i]);
            if (area > 150){
                 std::cout << " Area: " << contourArea(contours[i]) << std::endl;
            }
        }
            
        writer.write(frame);
        writer.write(frame_threshold);
        // Show the frames
        imshow(window_capture_name, frame);
        imshow(window_detection_name, frame_threshold);
        imshow(window_cont_name, cont);
        moveWindow(window_capture_name, 800,10);
        moveWindow(window_detection_name, 50,50);
        moveWindow(window_cont_name, 800,800);
        resizeWindow(window_capture_name, 950,550);
        resizeWindow(window_detection_name,600,600);
        resizeWindow(window_cont_name,950,550);

        // Press Space Bar to continue, ESC to exit
        char c = (char)cv::waitKey(0);
        if( c == 27 )
            break;

    }
    // When everything is done, release the video capture and writer objects
    cap.release();
    writer.release();

    // Close all the frames
    cv::destroyAllWindows();

    return;
}


//FIND CONTOURS OF BALL
Mat met(Mat frame, int thresh){
    Mat canny_output;
    Canny( frame, canny_output, thresh, thresh*2 );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    }
    
    return drawing;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "pm_proj1");
    //Create the node handles to establish the program as a ROS node
    ros::NodeHandle n_public;
    ros::NodeHandle n_private("~");
    
    n_private.param<int>("meanColor", meanColor, 0);
    n_private.param<int>("stdev", stdev , 0);
    n_private.param<string>("path", path , "../project_pm/src/pm_proj1/src/videoPlastic.mp4");

    method1(path, meanColor, stdev, ball_center.x, ball_center.y);

    n_public.advertise<geometry_msgs::Twist>("vloarder/frame", 1);
    n_public.advertise<geometry_msgs::Twist>("vloarder/ball_center", 1);
    //Spin - Infinite loop to ask ROS to read all pending callbacks
    ros::spin();
    return 0;

}






