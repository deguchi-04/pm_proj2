#include "vloader.cpp"

void cbPose(const turtlesim::Pose &msg)
{   
    if (!flag_first)
    {
        // Received center
        // FAZER direiro o subscriber
        ROS_WARN_STREAM("Inside callback!");
        ROS_WARN_STREAM("center=(" << msg.x << "," << msg.y << ")");
        flag_first = true;
    }
    geometry_msgs::Vector3 center;
    center.x = ball_center.x;
    center.y = ball_center.y;
    pub.publish(center);
}