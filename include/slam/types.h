#ifndef TYPES_H
#define TYPES_H

#include <gtsam/geometry/Pose3.h>

namespace slam {

template <class POSE>
struct Odometry {
    POSE odom;
    gtsam::SharedNoiseModel noise;
};

template <class POINT>
struct Measurement {
    POINT measurement;
    gtsam::SharedNoiseModel noise;
};


struct Landmark {
    unsigned long int id_gt;
    gtsam::Key key;
    gtsam::Pose3 pose;
};

} // namespace slam

#endif // TYPES_H