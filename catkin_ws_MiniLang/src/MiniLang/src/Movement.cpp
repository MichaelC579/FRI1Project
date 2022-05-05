#include <Movement.h>
#include <Decode.h>
#include <iostream>
#include <Eigen/Dense>
#include <string>

Transformer::Transformer(ros::NodeHandle &nh) {
    _sub = nh.subscribe("/azure_kinect/tag_pose", 1000, &Transformer::poseCallback, this);
    /*
        Your program will need to subscribe to "/azure_kinect/tag_pose"
        You should use a C++ style callback, the one in this class
        You should subscribe in your constructor, because your program needs
            to subscribe to the topic ONLY ONE TIME
    */
}

Transformer::~Transformer() {}

void Transformer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    Decode decoder;
    int ins = decoder.getNext();
    switch (ins) {
        
    }

    _br.sendTransform(_ts);
}
geometry_msg::TransformStamped moveForward(int distance, boolean feet, geometry_msgs::PoseStamped::ConstPtr &msg) {
    geometry_msgs::TransformStamped _ts;
    Eigen::Quaterniond r(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::MatrixXd robot(4,4);
    Eigen::MatrixXd rotation = robot.toRotationMatrix();
    robot.block<3,3>(0,0) = rotation;
    robot(0, 3) = msg->pose.position.x;
    robot(1, 3) = msg->pose.position.y;
    robot(2, 3) = msg->pose.position.z;
    robot(3, 3) = 1;

    Eigen::MatrixXd offset = Eigen::MatrixXd::Identity(4,4);
    if(feet) {
        offset(2,3) = (0.3048 * distance);
    } else {
        offset(2,3) = (0.0254 * distance);
    }
    Eigen::MatrixXd translate = robot * offset;
    _ts.header.frame_id = msg->header.frame_id;
    _ts.child_frame_id = "move_offset";
    _ts.transform.translation.x = translate(0,3);
    _ts.transform.translation.y = translate(1,3);
    _ts.transform.translation.z = translate(2,3);
    _ts.transform.rotation.x = msg->pose.orientation.x;
    _ts.transform.rotation.y = msg->pose.orientation.y;
    _ts.transform.rotation.z = msg->pose.orientation.z;
    _ts.transform.rotation.w = msg->pose.orientation.w;
    return _ts;
}

geometry_msg::TransformStamped rotate(int degrees, geometry_msgs::PoseStamped::ConstPtr &msg) {
    geometry_msgs::TransformStamped _ts;
    Eigen::Quaterniond r(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::MatrixXd robot(4,4);
    Eigen::MatrixXd rotation = robot.toRotationMatrix();
    robot.block<3,3>(0,0) = rotation;
    robot(0, 3) = msg->pose.position.x;
    robot(1, 3) = msg->pose.position.y;
    robot(2, 3) = msg->pose.position.z;
    robot(3, 3) = 1;

    Eigen::AngleAxisd rotate(degree * (M_PI/180), Eigen::Vector3d::UnitY());
    Eigen::MatrixXd rotated = robot.block<3,3>(0,0) * rotate;
    Eigen::Quaterniond turn(rotated.block<3,3>(0,0));
    _ts.header.frame_id = msg->header.frame_id;
    _ts.child_frame_id = "turn";
    _ts.transform.translation.x = msg->pose.position.x;
    _ts.transform.translation.y = msg->pose.position.y;
    _ts.transform.translation.z = msg->pose.position.z;
    _ts.transform.rotation.x = flipped.x();
    _ts.transform.rotation.y = flipped.y();
    _ts.transform.rotation.z = flipped.z();
    _ts.transform.rotation.w = flipped.w();
    return _ts;
}

geometry_msg::TransformStamped moveTo(int location, geometry_msgs::PoseStamped::ConstPtr &msg) {
    return geometry_msgs::TransformStamped _ts;
}

geometry_msg::TransformStamped follow(geometry_msgs::PoseStamped::ConstPtr &msg) {
    return geometry_msgs::TransformStamped _ts;
}


