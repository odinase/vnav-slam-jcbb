#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <functional>

#include <glog/logging.h>

#include "slam/slam.h"
#include "slam/slam_node.h"
#include "slam/utils.h"
#include "visualization_tools.h"

namespace slam
{

  SLAM_node::SLAM_node(const ros::NodeHandle &nh) : nh_(nh),
                                                    init_odom_(false),
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
    landmark_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("landmarks_positions", 10, true);

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

    gtsam::Rot3 R_oc = gtsam::Rot3::Rx(-M_PI/2)*gtsam::Rot3::Ry(M_PI/2);
    gtsam::Point3 t_oc = gtsam::Point3::Zero();

    T_oc_ = gtsam::Pose3(R_oc, t_oc); 
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

    geometry_msgs::TransformStamped odom_tf;
    // odom_tf.header.stamp = curr_time;
    odom_tf.header.frame_id = "world";
    odom_tf.child_frame_id = "body";
    int n = traj_msg.poses.size();
    odom_tf.transform.translation.x = traj_msg.poses[n-1].pose.position.x;
    odom_tf.transform.translation.y = traj_msg.poses[n-1].pose.position.y;
    odom_tf.transform.translation.z = traj_msg.poses[n-1].pose.position.z;
    odom_tf.transform.rotation = traj_msg.poses[n-1].pose.orientation;

    //send the transform
    pose_bc_.sendTransform(odom_tf);
  }

  void SLAM_node::publishLandmarkVisualization()
  {
    const auto landmark_poses = slam_.getLandmarkPoses();
    visualization_msgs::Marker landmark_pc_msg;
    std::vector<geometry_msgs::Point> landmark_points;
    for (const auto& landmark_pose : landmark_poses) {
      geometry_msgs::Point landmark_point;
      gtsam::Point3 lmk_gtsam_point = landmark_pose.translation();
      landmark_point.x = lmk_gtsam_point(0);
      landmark_point.y = lmk_gtsam_point(1);
      landmark_point.z = lmk_gtsam_point(2);
      landmark_points.push_back(landmark_point);
    }
    landmark_pc_msg.header.frame_id = "world";
    landmark_pc_msg.type = visualization_msgs::Marker::POINTS;
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0;
    landmark_pc_msg.color = color;
    landmark_pc_msg.points = landmark_points;
    landmark_pc_msg.scale.x = 0.25;
    landmark_pc_msg.scale.y = 0.25;
    landmark_pc_msg.scale.z = 0.25;

    landmark_viz_pub_.publish(landmark_pc_msg);
  }

  void SLAM_node::odomLandmarkDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &landmark_detections_msg, const nav_msgs::OdometryConstPtr &odom_msg)
  {
    static int scan =0;
    std::cout << "Doing scan " << scan++ << "\n";
    // std::cout << "Number of measurements: " << landmark_detections_msg->detections.size() << "\n";
    // Simply change typings of things into SLAM compatible stuff, then pass on to frontend of SLAM
    // if (scan > 30) {
    //   ros::shutdown();
    // }
    gtsam::Pose3 odom = convertOdomToRelative(rosPoseToGtsamPose(odom_msg->pose.pose));
    // std::cout << "odom:\nt:\n" << odom.translation() << "\nrpy:\n" << odom.rotation().rpy() << "\n";
    gtsam::FastVector<gtsam::Pose3> measurements;
    for (const auto &detection : landmark_detections_msg->detections)
    {
      // TODO: Should here store gt id for later
      gtsam::Pose3 lmk_measurement = T_oc_*rosPoseToGtsamPose(detection.pose.pose.pose);
      measurements.push_back(lmk_measurement);
    }

    slam_.processOdomMeasurementScan(odom, measurements);

    publishTrajectory();
    publishLandmarkVisualization();
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