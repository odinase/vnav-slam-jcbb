#ifndef KNOWN_DATA_ASSOCIATION_H
#define KNOWN_DATA_ASSOCIATION_H

#include <vector>
#include <queue>
#include <limits>
#include <eigen3/Eigen/Core>
#include <numeric>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <unordered_map>

#include <gtsam/base/FastVector.h>
#include <gtsam/geometry/Pose3.h>

// Seems dirty to "cross-include" like this, bu w/e
#include "jcbb/Hypothesis.h"

namespace gt
{

using jcbb::Hypothesis;
using jcbb::Association;

class KnownDataAssociation {

private:
        const gtsam::Values &estimates_;
        const gtsam::Marginals &marginals_;
        const gtsam::FastVector<int> &measured_apriltags_;
        const gtsam::FastVector<gtsam::Pose3> &measurements_;
        const gtsam::noiseModel::Diagonal::shared_ptr meas_noise_;
        gtsam::Pose3 x_pose_;
        gtsam::Key x_key_;
        int lmk_key_;

        // For keeping track internally of associated landmarks and apriltags
        std::unordered_map<int, gtsam::Key>* apriltag_id_lmk_;

public:
        KnownDataAssociation(
                const gtsam::Values &estimates,
                const gtsam::Marginals &marginals_,
                const gtsam::FastVector<gtsam::Pose3> &measurements,
                const gtsam::noiseModel::Diagonal::shared_ptr &meas_noise,
                const gtsam::FastVector<int> &measured_apriltags,
                std::unordered_map<int, gtsam::Key>* apriltag_id_lmk
        );
        double joint_compatability(const Hypothesis &h) const;
        double individual_compatability(const Association &a) const;
        Hypothesis associate();
};

    
} // namespace gt

#endif // KNOWN_DATA_ASSOCIATION_H