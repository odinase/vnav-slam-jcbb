#ifndef SLAM_NODE_H
#define SLAM_NODE_H

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Path.h>
#include <gtsam/base/FastSet.h>
#include <set>

#include "slam/slam.h"
#include "slam/types.h"

namespace slam
{

  class SLAM_node
  {

  private:
    ros::NodeHandle nh_;

    message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> sf_landmark_detections_;
    message_filters::Subscriber<nav_msgs::Odometry> sf_odom_;

    typedef message_filters::
        sync_policies::ApproximateTime<apriltag_ros::AprilTagDetectionArray, nav_msgs::Odometry>
            OdomLandmarkDetectionsSyncPolicy;

    message_filters::Synchronizer<OdomLandmarkDetectionsSyncPolicy> sync_;

    ros::Publisher latest_pose_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher landmark_poses_pub_;
    ros::Publisher landmark_viz_pub_;
    tf2_ros::TransformBroadcaster pose_bc_;

    SLAM slam_;
    bool init_odom_;
    bool should_log_;
    gtsam::Pose3 prev_odom_;
    gtsam::Pose3 T_oc_;
    std::set<int> detected_apriltags_;
    int number_of_messages_to_process_;
    int processed_messages_;

    void odomLandmarkDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &landmark_detections_msg, const nav_msgs::OdometryConstPtr &odom_msg);
    gtsam::Pose3 convertOdomToRelative(const gtsam::Pose3 &raw_odom);

  public:
    SLAM_node(const ros::NodeHandle &nh);
    void publishTrajectory();
    void publishLandmarkPoses();
    void publishLandmarkVisualization();
    void logData() const;
    bool should_log() const { return should_log_; }
    bool processed_all_messages() const {return processed_messages_ >= number_of_messages_to_process_; }
  };

} // namespace slam

#endif // SLAM_NODE_H