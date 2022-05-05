#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class Movement {
protected:
    ros::Subscriber _sub;
    tf2_ros::TransformBroadcaster _br;
public:
    /*
        Your constructor should take a ros::NodeHandle REFERENCE (or POINTER) as a parameter
        If it is not passed as a reference or pointer, your program will not work.
    */
    Movement(ros::NodeHandle &nh);
    ~Movement();

    //You will need to make a function that is a callback which subscribes to geometry_msgs::PoseStamped
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    geometry_msg::TransformStamped moveForward(int distance, boolean feet, geometry_msgs::PoseStamped::ConstPtr &msg);
    geometry_msg::TransformStamped rotate(int degrees, geometry_msgs::PoseStamped::ConstPtr &msg);
    geometry_msg::TransformStamped moveTo(int location, geometry_msgs::PoseStamped::ConstPtr &msg);
    geometry_msg::TransformStamped follow(geometry_msgs::PoseStamped::ConstPtr &msg);
};

#endif