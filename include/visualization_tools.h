#ifndef VISUALIZATION_TOOLS_H
#define VISUALIZATION_TOOLS_H

// GTSAM
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

// ROS headers.
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>





struct PointColor {
  PointColor(float r = 0.0, float g = 1.0, float b = 0.0, float a = 1.0)
      : r_(r), g_(g), b_(b), a_(a) {}
  float r_;
  float g_;
  float b_;
  float a_;
};

// Color for a line.
struct LineColor {
  LineColor(float r = 0.0, float g = 1.0, float b = 0.0, float a = 1.0)
      : r_(r), g_(g), b_(b), a_(a) {}
  float r_;
  float g_;
  float b_;
  float a_;
};

// Color for a trajectory.
struct TrajectoryColor {
  TrajectoryColor(const PointColor& point_color = PointColor(),
                  const LineColor& line_color = LineColor())
      : point_color_(point_color), line_color_(line_color) {}
  PointColor point_color_;
  LineColor line_color_;
};







void DrawPoint3(const ros::Publisher& marker_pub,
                std::vector<gtsam::Point3>& landmarks,
                const TrajectoryColor& color = TrajectoryColor()) {
  static constexpr char kFrameId[] = "world";

  visualization_msgs::Marker landmark_markers;
  landmark_markers.header.frame_id = kFrameId;
  landmark_markers.header.stamp = ros::Time::now();
  landmark_markers.ns = "landmark";
  landmark_markers.action = visualization_msgs::Marker::ADD;
  landmark_markers.pose.orientation.w = 1.0;
  landmark_markers.id = 0;
  landmark_markers.type = visualization_msgs::Marker::POINTS;

  landmark_markers.scale.x = 0.1;
  landmark_markers.scale.y = 0.1;

  // landmark_markers
  landmark_markers.color.r = color.point_color_.r_;
  landmark_markers.color.g = color.point_color_.g_;
  landmark_markers.color.b = color.point_color_.b_;
  landmark_markers.color.a = color.point_color_.a_;

  // Loop over the trajectory.
  for (const auto& landmark : landmarks) {
    geometry_msgs::Point pt;
    pt.x = landmark.x();
    pt.y = landmark.y();
    pt.z = landmark.z();
    landmark_markers.points.push_back(pt);
  }

  // Publish
  marker_pub.publish(landmark_markers);
}

#endif // VISUALIZATION_TOOLS_H