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

    using gtsam::symbol_shorthand::X;
    using gtsam::symbol_shorthand::L;

    enum class AssociationMethod {
        JCBB,
        ML,
        KnownDataAssociation
    };

    class SLAM
    {
    private:
        gtsam::NonlinearFactorGraph graph_;
        gtsam::Values estimates_;
        gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
        gtsam::noiseModel::Diagonal::shared_ptr meas_noise_;
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;

        // We assume that JCBB is not perfect, so the same apriltag can be associated to multiple landmarks
        gtsam::FastMap<int, gtsam::FastVector<gtsam::Key>> apriltag_lmk_assos_;
        gtsam::FastVector<jcbb::Hypothesis> hypotheses_;

        unsigned long int latest_pose_key_;
        gtsam::Pose3 latest_pose_;
        unsigned long int latest_landmark_key_;

        void incrementLatestPoseKey() { latest_pose_key_++; }
        void incrementLatestLandmarkKey() { latest_landmark_key_++; }

        void addOdom(const gtsam::Pose3 &odom);
        gtsam::FastVector<gtsam::Pose3> predictLandmarks() const;

        // For keeping track internally of associated landmarks and apriltags
        std::unordered_map<int, gtsam::Key> apriltag_id_lmk_;

        int optimization_rate_;
        double ic_prob_;
        double jc_prob_;

        AssociationMethod association_method_;

    public:
        SLAM();
        // void addPose(const gtsam::Pose3& x_pose);
        // Assumes the landmark is detected in the current pose.
        void addLandmark(const Landmark &lmk);

        // We need to initialize the graph with priors on the first pose and landmark
        void optimize();
        const gtsam::Values &currentEstimates() const;
        void processOdomMeasurementScan(const gtsam::Pose3& odom, const gtsam::FastVector<gtsam::Pose3> &measurements, const gtsam::FastVector<int>& measured_apriltags);
        void initialize(double ic_prob, double jc_prob, int optimization_rate, const std::vector<double>& odom_noise, const std::vector<double>& meas_noise, const std::vector<double>& prior_noise, AssociationMethod association_method);
        gtsam::FastVector<gtsam::Pose3> getTrajectory() const;
        gtsam::FastVector<gtsam::Pose3> getLandmarkPoses() const;
        const gtsam::FastMap<int, gtsam::FastVector<gtsam::Key>>& getApriltagLandmarkAssos() const { return apriltag_lmk_assos_; }  
        const gtsam::FastVector<jcbb::Hypothesis>& getChosenHypotheses() const { return hypotheses_; }
    };

} // namespace slam

#endif // SLAM_H