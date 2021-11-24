#include "slam/slam.h"
#include "jcbb/jcbb.h"
#include "jcbb/Hypothesis.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <iostream>
#include <chrono>

namespace slam
{

  SLAM::SLAM()
      : latest_pose_key_(0),
        latest_landmark_key_(0)
  {
    // Add prior on first pose

    // TODO: Should not be here, but oh well
    gtsam::Vector6 sigmas;
    double x_std = 1e-3;
    double y_std = 1e-3;
    double z_std = 1e-3;
    double roll_std = 1e-3;
    double pitch_std = 1e-3;
    double yaw_std = 1e-3;
    sigmas << x_std, y_std, z_std, roll_std, pitch_std, yaw_std;
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(latest_pose_key_), gtsam::Pose3(), prior_noise));
    estimates_.insert(X(latest_pose_key_), gtsam::Pose3());

    // Let's use the same sigmas for everything for now
    odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    meas_noise_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  }

  void SLAM::initialize(double ic_prob, double jc_prob, int optimization_rate)
  {
    ic_prob_ = ic_prob;
    jc_prob_ = jc_prob;
    optimization_rate_ = optimization_rate;
    std::cout << "Using ic_prob " << ic_prob << ", jc_prob " << jc_prob << ", optimization_rate " << optimization_rate << "\n";
  }

  gtsam::FastVector<gtsam::Pose3> SLAM::getTrajectory() const
  {
    gtsam::FastVector<gtsam::Pose3> trajectory;
    for (int i = 0; i < latest_pose_key_; i++)
    {
      trajectory.push_back(estimates_.at<gtsam::Pose3>(X(i)));
    }
    return trajectory;
  }

  gtsam::FastVector<gtsam::Point3> SLAM::getLandmarkPositions() const
  {
    gtsam::FastVector<gtsam::Point3> landmarks;
    for (int i = 0; i < latest_landmark_key_; i++) {
      landmarks.push_back(estimates_.at<gtsam::Pose3>(L(i)).translation());
    }
    return landmarks;
  }

  void SLAM::processOdomMeasurementScan(const gtsam::Pose3 odom, const gtsam::FastVector<gtsam::Pose3> &measurements)
  {
    // std::cout << "\n\nMEASUREMENTS!!!\n\n";
    static int num = 1;
    // Add odom to graph
    addOdom(odom);
    if (num % optimization_rate_ != 0)
    {
      // std::cout << "Skipping! num is " << num << "\n";
      num++;
      return;
    }

    num = 0;

    // std::cout << "Doing it all!\n";

    auto start = std::chrono::high_resolution_clock::now();

    // Predict measurements from stored landmarks (TODO: drop all landmarks not within field of view?)
    // gtsam::FastVector<gtsam::Pose3> predicted_measurements = predictLandmarks();

    // I don't like calling a function with "unsafe" in it, will have to think of better way of doing this...
    // gtsam::Marginals marginals = gtsam::Marginals(isam_.getFactorsUnsafe(), current_estimates_);
    gtsam::Marginals marginals = gtsam::Marginals(graph_, estimates_);

    // Run all through JCBB
    jcbb::JCBB jcbb_(estimates_, marginals, measurements, meas_noise_, gtsam::Vector::Zero(3), ic_prob_, jc_prob_);
    jcbb::Hypothesis h = jcbb_.jcbb();
    // std::cout << "\n\nCompleted JCBB! Hypothesis:\n";
    const auto &assos = h.associations();
    for (int i = 0; i < assos.size(); i++)
    {
      jcbb::Association::shared_ptr a = assos[i];
      if (a->associated())
      {
        std::cout << "associated meas " << a->measurement << " with landmark " << gtsam::symbolIndex(*a->landmark) << "\n";
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(latest_pose_key_), *a->landmark, measurements[i], meas_noise_));
      }
      else
      {
        // std::cout << "meas " << a->measurement << " unassociated\n";
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(latest_pose_key_), L(latest_landmark_key_), measurements[i], meas_noise_));
        estimates_.insert(L(latest_landmark_key_), measurements[i]);
        // std::cout << "Added " << gtsam::symbolChr(L(latest_landmark_key_)) << gtsam::symbolIndex(L(latest_landmark_key_)) << "\n";
        incrementLatestLandmarkKey();
      }
      // We just inserted our first landmark, set prior
      if (latest_landmark_key_ == 1)
      {
        // TODO: Should not be here, but oh well
        gtsam::Vector6 sigmas;
        double x_std = 1e-3;
        double y_std = 1e-3;
        double z_std = 1e-3;
        double roll_std = 1e-3;
        double pitch_std = 1e-3;
        double yaw_std = 1e-3;
        sigmas << x_std, y_std, z_std, roll_std, pitch_std, yaw_std;
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(L(0), measurements[i], prior_noise));
      }
    }
    // std::cout << "\n\n";

    // std::cout << "CAMEE GEERERER!!!\n";
    // isam_.update(graph_, estimates_);
    // for (int i = 0; i < 20; i++) {
    //   isam_.update();
    // }

    // graph_.resize(0);
    // estimates_.clear();

    // Optimize the graph.
    // if (num % optimization_rate == 0) {

    gtsam::LevenbergMarquardtParams params;
    // params.setVerbosity("ERROR");
    params.setAbsoluteErrorTol(1e-08);
    estimates_ =
        gtsam::LevenbergMarquardtOptimizer(graph_, estimates_, params).optimize();
    // }

    auto stop = std::chrono::high_resolution_clock::now();
    // std::cout << "Spent " << std::chrono::duration_cast<std::chrono::duration<double>>(stop - start).count() << " s in function.\n";
    // Add factors
    // If not initialize, do that with two factors
  }

  void SLAM::addOdom(const gtsam::Pose3 &odom)
  {
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(latest_pose_key_), X(latest_pose_key_ + 1), odom, odom_noise_));
    gtsam::Pose3 this_pose = latest_pose_ * odom;
    estimates_.insert(X(latest_pose_key_ + 1), this_pose);
    // std::cout << "Added " << gtsam::symbolChr(X(latest_pose_key_ + 1)) << gtsam::symbolIndex(X(latest_pose_key_ + 1)) << "\n";
    latest_pose_ = this_pose;
    incrementLatestPoseKey();
  }

  gtsam::FastVector<gtsam::Pose3> SLAM::predictLandmarks() const
  {
    gtsam::KeyList landmark_keys = estimates_.filter(gtsam::Symbol::ChrTest('l')).keys();
    if (landmark_keys.size() == 0)
    {
      return {};
    }
    gtsam::FastVector<gtsam::Pose3> predicted_measurements;
    for (const auto &lmk : landmark_keys)
    {
      predicted_measurements.push_back(estimates_.at<gtsam::Pose3>(lmk));
    }

    return predicted_measurements;
  }
} // namespace slam