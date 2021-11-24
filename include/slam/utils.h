#ifndef UTILS_H
#define UTILS_H

#include <gtsam/geometry/Pose3.h>
#include <geometry_msgs/Pose.h>

namespace slam
{
    gtsam::Pose3 rosPoseToGtsamPose(const geometry_msgs::Pose &pose)
    {
        gtsam::Point3 t(pose.position.x, pose.position.y,
                        pose.position.z);

        gtsam::Rot3 R =
            gtsam::Rot3::Quaternion(pose.orientation.w, pose.orientation.x,
                                    pose.orientation.y, pose.orientation.z);

        return gtsam::Pose3(R, t);
    }
} // namespace slam

#endif // UTILS_H