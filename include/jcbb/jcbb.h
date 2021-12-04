#ifndef JCBB_H
#define JCBB_H

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

#include "MarginalMocks.h"
#include "jcbb/Hypothesis.h"


namespace jcbb
{
    using State = gtsam::Pose3;
    using Landmark = gtsam::Pose3;
    using Measurement = gtsam::Pose3;
    template <class T>
    using FastMinHeap = std::priority_queue<T, gtsam::FastVector<T>, std::greater<T>>;
    double chi2inv(double p, unsigned int dim);

    class JCBB
    {
    public:
        JCBB(const gtsam::Values &estimates, const Marginals &marginals_, const gtsam::FastVector<Measurement> &measurements, const gtsam::noiseModel::Diagonal::shared_ptr &meas_noise, double ic_prob, double jc_prob);
        double joint_compatability(const Hypothesis &h) const;
        double individual_compatability(const Association &a) const;
        Hypothesis jcbb() const;

    private:
        const gtsam::Values &estimates_;
        const Marginals &marginals_;
        const gtsam::FastVector<Measurement> &measurements_;
        const gtsam::noiseModel::Diagonal::shared_ptr meas_noise_;
        State x_pose_;
        gtsam::Key x_key_;
        gtsam::KeyList landmark_keys_;
        double ic_prob_;
        double jc_prob_;

        void push_successors_on_heap(FastMinHeap<Hypothesis>* min_heap, const Hypothesis &h) const;
        bool feasible(const Hypothesis &h) const;
        bool prunable(const Hypothesis &h, const Hypothesis &best) const;
        // std::unordered_map<>
    };

} // namespace jcbb

#endif // JCBB_H