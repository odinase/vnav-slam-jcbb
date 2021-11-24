#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <functional>

#include <glog/logging.h>

#include "slam/slam.h"
#include "slam/slam_node.h"
#include "slam/utils.h"
#include "visualization_tools.h"

namespace slam
{

  SLAM_node::SLAM_node(const ros::NodeHandle &nh) : nh_(nh),
                                                    sf_landmark_detections_(nh_, "/tag_detections", 1000),
                                                    sf_odom_(nh_, "/zed/zed_node/odom", 1000),
                                                    sync_(OdomLandmarkDetectionsSyncPolicy(1000), sf_landmark_detections_, sf_odom_)
  {
    ROS_INFO("Spinning up SLAM node.");
    sync_.registerCallback(std::bind(&SLAM_node::odomLandmarkDetectionsCallback,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2));

    latest_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovariance>("slam_pose", 1);
    landmark_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("landmark_poses", 1);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("trajectory", 1);
    landmark_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/landmarks_positions", 10, true);

    double ic_prob, jc_prob;

    if (!nh.getParam("ic_prob", ic_prob))
    {
      ROS_ERROR_STREAM("Unable to read ic_prob, shutting down");
      ros::shutdown();
    }

    if (!nh.getParam("jc_prob", jc_prob))
    {
      ROS_ERROR_STREAM("Unable to read jc_prob, shutting down");
      ros::shutdown();
    }

    int optimization_rate;

    if (!nh.getParam("optimization_rate", optimization_rate))
    {
      ROS_ERROR_STREAM("Unable to read optimization_rate, shutting down");
      ros::shutdown();
    }
    slam_.initialize(ic_prob, jc_prob, optimization_rate);
  }

  void SLAM_node::publishTrajectory()
  {
    gtsam::FastVector<gtsam::Pose3> trajectory = slam_.getTrajectory();
    nav_msgs::Path traj_msg;
    traj_msg.header.frame_id = "world";
    for (const auto pose : trajectory)
    {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = "world";
      pose_msg.pose.position.x = pose.x();
      pose_msg.pose.position.y = pose.y();
      pose_msg.pose.position.z = pose.z();
      pose_msg.pose.orientation.w = pose.rotation().quaternion()(0);
      pose_msg.pose.orientation.x = pose.rotation().quaternion()(1);
      pose_msg.pose.orientation.y = pose.rotation().quaternion()(2);
      pose_msg.pose.orientation.z = pose.rotation().quaternion()(3);
      traj_msg.poses.push_back(pose_msg);
    }

    trajectory_pub_.publish(traj_msg);
  }

  void SLAM_node::publishLandmarkPoses()
  {
  }

  void SLAM_node::odomLandmarkDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &landmark_detections_msg, const nav_msgs::OdometryConstPtr &odom_msg)
  {
    // Simply change typings of things into SLAM compatible stuff, then pass on to frontend of SLAM
    gtsam::Pose3 odom = convertOdomToRelative(rosPoseToGtsamPose(odom_msg->pose.pose));
    gtsam::FastVector<gtsam::Pose3> measurements;
    for (const auto &detection : landmark_detections_msg->detections)
    {
      // TODO: Should here store gt id for later
      gtsam::Pose3 lmk_measurement = rosPoseToGtsamPose(detection.pose.pose.pose);
      measurements.push_back(lmk_measurement);
    }

    slam_.processOdomMeasurementScan(odom, measurements);

    publishTrajectory();
  }

  gtsam::Pose3 SLAM_node::convertOdomToRelative(const gtsam::Pose3 &raw_odom)
  {
    gtsam::Pose3 odom = gtsam::Pose3::identity();
    if (!init_odom_)
    {
      prev_odom_ = raw_odom;
      init_odom_ = true;
    }
    else
    {
      odom = prev_odom_.inverse() * raw_odom;
    }
    return odom;
  }

} // namespace slam

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "slam_node");
  ROS_INFO("Starting.");

  ros::NodeHandle nh;
  slam::SLAM_node slam_node(nh);

  // ROS spin until process killed.
  while (ros::ok())
  {
    ros::spin();
  }
}