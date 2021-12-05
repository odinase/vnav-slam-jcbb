#ifndef MAXIMUM_LIKELIHOOD_H
#define MAXIMUM_LIKELIHOOD_H

#include <vector>
#include <queue>
#include <limits>
#include <eigen3/Eigen/Core>
#include <numeric>
#include <unordered_set>
#include <unordered_map>
#include <iostream>

#include <gtsam/base/FastVector.h>
#include <gtsam/geometry/Pose3.h>

// Seems dirty to "cross-include" like this, bu w/e
#include "jcbb/Hypothesis.h"

namespace ml {

using jcbb::Hypothesis;
using jcbb::Association;

class MaximumLikelihood {

private:
        const gtsam::Values &estimates_;
        const gtsam::Marginals &marginals_;
        const gtsam::FastVector<gtsam::Pose3> &measurements_;
        const gtsam::noiseModel::Diagonal::shared_ptr meas_noise_;
        gtsam::Pose3 x_pose_;
        gtsam::Key x_key_;
        gtsam::KeyList landmark_keys_;
        double ic_prob_;

public:
        MaximumLikelihood(const gtsam::Values &estimates, const gtsam::Marginals &marginals_, const gtsam::FastVector<gtsam::Pose3> &measurements, const gtsam::noiseModel::Diagonal::shared_ptr &meas_noise, double ic_prob);
        double joint_compatability(const Hypothesis &h) const;
        double individual_compatability(const Association &a) const;
        Hypothesis associate() const;
};

} // namespace ml


#endif // MAXIMUM_LIKELIHOOD_H