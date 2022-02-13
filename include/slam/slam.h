#ifndef SLAM_H
#define SLAM_H

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <vector>

#include "slam/types.h"
#include "jcbb/Hypothesis.h"

namespace slam
{

    using gtsam::symbol_shorthand::L;
    using gtsam::symbol_shorthand::X;

    enum class AssociationMethod
    {
        JCBB,
        ML,
        KnownDataAssociation
    };

    template<class POSE, class POINT>
    class SLAM
    {
    private:
        gtsam::NonlinearFactorGraph graph_;
        gtsam::Values estimates_;
        gtsam::noiseModel::Diagonal::shared_ptr pose_prior_noise_;
        gtsam::noiseModel::Diagonal::shared_ptr lmk_prior_noise_;

        gtsam::FastVector<jcbb::Hypothesis> hypotheses_;

        unsigned long int latest_pose_key_;
        POSE latest_pose_;
        unsigned long int latest_landmark_key_;

        void incrementLatestPoseKey() { latest_pose_key_++; }
        void incrementLatestLandmarkKey() { latest_landmark_key_++; }

        void addOdom(const Odometry<POSE> &odom);
        gtsam::FastVector<POINT> predictLandmarks() const;

        // For keeping track internally of associated landmarks and apriltags
        // std::unordered_map<int, gtsam::Key> apriltag_id_lmk_;

        int optimization_rate_;
        double ic_prob_;

        // AssociationMethod association_method_;

    public:
        SLAM();

        // We need to initialize the graph with priors on the first pose and landmark
        void optimize();
        const gtsam::Values &currentEstimates() const;
        void processOdomMeasurementScan(const Odometry<POSE> &odom, const gtsam::FastVector<Measurement<POINT>> &measurements);
        void initialize(double ic_prob, int optimization_rate, const gtsam::Vector &pose_prior_noise, const gtsam::Vector &lmk_prior_noise);
        gtsam::FastVector<POSE> getTrajectory() const;
        gtsam::FastVector<POINT> getLandmarkPoints() const;
        const gtsam::FastVector<jcbb::Hypothesis> &getChosenHypotheses() const { return hypotheses_; }
        // AssociationMethod getAssociationMethod() const { return association_method_; }
    };

} // namespace slam

#include "slam/slam.hxx"

#endif // SLAM_H