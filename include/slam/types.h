#ifndef TYPES_H
#define TYPES_H

#include <gtsam/geometry/Pose3.h>

struct Landmark {
    unsigned long int id_gt;
    gtsam::Key key;
    gtsam::Pose3 pose;
};

#endif // TYPES_H