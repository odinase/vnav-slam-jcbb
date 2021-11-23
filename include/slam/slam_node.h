#ifndef SLAM_NODE_H
#define SLAM_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Path.h>
#include "slam/slam.h"
#include "slam/types.h"

namespace slam {

class SLAM_node
{

public:
    SlamNode(const ros::NodeHandle &nh);
    void publishTrajectory();

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber landmark_detections_sub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher landmark_pos_pub_;
    SLAM slam_;

    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
    void landmarkDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& landmark_detections_msg);
};

} // namespace slam

#endif // SLAM_NODE_H