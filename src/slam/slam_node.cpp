#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

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
#include <limits>

#include <functional>
#include <fstream>

#include <glog/logging.h>

#include "slam/slam.h"
#include "slam/slam_node.h"
#include "slam/utils.h"
#include "visualization_tools.h"

namespace slam
{

  SLAM_node::SLAM_node(const ros::NodeHandle &nh) : nh_(nh),
                                                    init_odom_(false),
                                                    processed_messages_(0),
                                                    sf_landmark_detections_(nh_, "/tag_detections", 6166),
                                                    sf_odom_(nh_, "/zed/zed_node/odom", 6166),
                                                    sync_(OdomLandmarkDetectionsSyncPolicy(6166), sf_landmark_detections_, sf_odom_)
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

    std::vector<double> odom_noise, meas_noise, prior_noise;

    if (!nh.getParam("odom_noise", odom_noise))
    {
      ROS_ERROR_STREAM("Unable to read odom_noise, shutting down");
      ros::shutdown();
    }

    if (!nh.getParam("meas_noise", meas_noise))
    {
      ROS_ERROR_STREAM("Unable to read meas_noise, shutting down");
      ros::shutdown();
    }

    if (!nh.getParam("prior_noise", prior_noise))
    {
      ROS_ERROR_STREAM("Unable to read prior_noise, shutting down");
      ros::shutdown();
    }

    int association_method_int;
    AssociationMethod association_method;

    if (!nh.getParam("association_method", association_method_int))
    {
      ROS_WARN("Unable to read association_method, using JCBB");
      association_method = AssociationMethod::JCBB;
    }

    switch (association_method_int)
    {
    case 0:
    {
      association_method = AssociationMethod::JCBB;
      break;
    }
    case 1:
    {
      association_method = AssociationMethod::ML;
      break;
    }
    default:
    {
      association_method = AssociationMethod::JCBB;
      break;
    }
    }

    slam_.initialize(ic_prob, jc_prob, optimization_rate, odom_noise, meas_noise, prior_noise, association_method);

    bool should_log = false;

    if (!nh.getParam("should_log", should_log))
    {
      ROS_WARN("Unable to read should_log, won't log");
      should_log = false;
    }

    should_log_ = should_log;

    int number_of_messages_to_process = 0;

    if (!nh.getParam("number_of_messages_to_process", number_of_messages_to_process))
    {
      ROS_WARN("Unable to read number_of_messages_to_process, will process infinitely many messages");
    }
    if (number_of_messages_to_process < 1)
    {
      number_of_messages_to_process = std::numeric_limits<int>::max();
    }

    number_of_messages_to_process_ = number_of_messages_to_process;

    gtsam::Rot3 R_oc = gtsam::Rot3::Rx(-M_PI / 2) * gtsam::Rot3::Ry(M_PI / 2);
    gtsam::Point3 t_oc = gtsam::Point3::Zero();

    T_oc_ = gtsam::Pose3(R_oc, t_oc);
  }

  void SLAM_node::publishTrajectory()
  {
    gtsam::FastVector<gtsam::Pose3> trajectory = slam_.getTrajectory();
    if (trajectory.size() == 0)
    {
      ROS_INFO("No trajectory to publish");
      return;
    }
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
    odom_tf.transform.translation.x = traj_msg.poses[n - 1].pose.position.x;
    odom_tf.transform.translation.y = traj_msg.poses[n - 1].pose.position.y;
    odom_tf.transform.translation.z = traj_msg.poses[n - 1].pose.position.z;
    odom_tf.transform.rotation = traj_msg.poses[n - 1].pose.orientation;

    // send the transform
    pose_bc_.sendTransform(odom_tf);
  }

  void SLAM_node::publishLandmarkVisualization()
  {
    const auto landmark_poses = slam_.getLandmarkPoses();
    visualization_msgs::Marker landmark_pc_msg;
    landmark_pc_msg.ns = "landmarks";
    std::vector<geometry_msgs::Point> landmark_points;
    for (const auto &landmark_pose : landmark_poses)
    {
      geometry_msgs::Point landmark_point;
      gtsam::Point3 lmk_gtsam_point = landmark_pose.translation();
      landmark_point.x = lmk_gtsam_point(0);
      landmark_point.y = lmk_gtsam_point(1);
      landmark_point.z = lmk_gtsam_point(2);
      landmark_points.push_back(landmark_point);
    }
    landmark_pc_msg.header.frame_id = "world";
    landmark_pc_msg.type = visualization_msgs::Marker::CUBE_LIST;
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.2;
    color.b = 0.0;
    color.a = 1.0;
    landmark_pc_msg.color = color;
    landmark_pc_msg.points = landmark_points;
    landmark_pc_msg.scale.x = 0.25;
    landmark_pc_msg.scale.y = 0.25;
    landmark_pc_msg.scale.z = 0.25;

    // ROS_INFO_STREAM("In total " << detected_apriltags_.size() << " aptiltags detected, and " << landmark_poses.size() << " landmarks\n");

    landmark_viz_pub_.publish(landmark_pc_msg);
  }

  void SLAM_node::odomLandmarkDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &landmark_detections_msg, const nav_msgs::OdometryConstPtr &odom_msg)
  {
    gtsam::Pose3 odom = convertOdomToRelative(rosPoseToGtsamPose(odom_msg->pose.pose));
    gtsam::FastVector<gtsam::Pose3> measurements;
    gtsam::FastVector<int> measured_apriltags;
    for (const auto &detection : landmark_detections_msg->detections)
    {
      detected_apriltags_.insert(detection.id[0]);
      measured_apriltags.push_back(detection.id[0]);

      gtsam::Pose3 lmk_measurement = T_oc_ * rosPoseToGtsamPose(detection.pose.pose.pose);
      measurements.push_back(lmk_measurement);
    }

    slam_.processOdomMeasurementScan(odom, measurements, measured_apriltags);

    publishTrajectory();
    publishLandmarkVisualization();

    processed_messages_++;
    ROS_INFO_STREAM("Processed " << processed_messages_ << " / " << number_of_messages_to_process_ << " messages");
  }

  gtsam::Pose3 SLAM_node::convertOdomToRelative(const gtsam::Pose3 &raw_odom)
  {
    logOdom(raw_odom);

    gtsam::Pose3 odom = gtsam::Pose3::identity();
    if (!init_odom_)
    {
      init_odom_ = true;
    }
    else
    {
      odom = prev_odom_.inverse() * raw_odom;
    }
    prev_odom_ = raw_odom;
    return odom;
  }

  void SLAM_node::logOdom(const gtsam::Pose3 &odom)
  {
    if (!init_odom_)
    {
      if (raw_odoms_.size() > 0)
      {
        ROS_ERROR("raw_odoms_ should be empty before init!");
        ros::shutdown();
      }
      raw_odoms_.push_back(gtsam::Pose3());
    }
    else
    {
      gtsam::Pose3 rel_odom = prev_odom_.inverse() * odom;
      gtsam::Pose3 new_pose = raw_odoms_.back() * rel_odom;
      raw_odoms_.push_back(new_pose);
    }
  }

  void SLAM_node::logData() const
  {
    std::string logging_path = ros::package::getPath("slam") + "/logs";
    std::fstream path_file(logging_path + "/path.txt", std::ios::out);
    gtsam::FastVector<gtsam::Pose3> trajectory = slam_.getTrajectory();
    // We project the pose onto 2D since everything is basically flat.
    for (const auto &pose : trajectory)
    {
      path_file << pose.translation()(0) << " " << pose.translation()(1) << " " << pose.rotation().yaw() << "\n";
    }
    path_file.close();

    std::fstream landmarks_file(logging_path + "/landmarks.txt", std::ios::out);
    gtsam::FastVector<gtsam::Pose3> landmark_poses = slam_.getLandmarkPoses();
    for (const auto &lmk : landmark_poses)
    {
      landmarks_file << lmk.translation()(0) << " " << lmk.translation()(1) << "\n";
    }
    landmarks_file.close();

    std::fstream raw_odom_file(logging_path + "/odom.txt", std::ios::out);
    for (const auto &odom : raw_odoms_)
    {
      raw_odom_file << odom.translation()(0) << " " << odom.translation()(1) << " " << odom.rotation().yaw() << "\n";
    }
    raw_odom_file.close();

    std::fstream apriltag_lmk_assos_file(logging_path + "/april_lmk_assos.txt", std::ios::out);
    const auto &apriltag_lmk_assos = slam_.getApriltagLandmarkAssos();
    for (const auto &[apriltag_id, lmk_ids] : apriltag_lmk_assos)
    {
      apriltag_lmk_assos_file << apriltag_id << " ";
      for (const auto &lmk : lmk_ids)
      {
        apriltag_lmk_assos_file << gtsam::symbolIndex(lmk) << " ";
      }
      apriltag_lmk_assos_file << "\n";
    }
    apriltag_lmk_assos_file.close();

    std::fstream nis_consistency_file(logging_path + "/nis_consistency.txt", std::ios::out);
    const auto &hypotheses = slam_.getChosenHypotheses();

    for (const auto &h : hypotheses)
    {
      nis_consistency_file << h.get_nis() << " " << h.num_associations() * gtsam::Pose3::dimension << "\n";
    }

    nis_consistency_file.close();
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

  // ROS spin until process killed or all desired messages are processed.
  while (ros::ok() && !slam_node.processed_all_messages())
  {
    // Process just the oldest callback, not all that are accumulated
    ros::getGlobalCallbackQueue()->callOne(ros::WallDuration(0));
  }
  ROS_INFO("Processed all messages!");

  if (slam_node.should_log())
  {
    ROS_INFO("Logging data!");
    slam_node.logData();
    ROS_INFO("Logging data complete!");
  }
}