#include "slam/slam.h"
#include "jcbb/jcbb.h"
#include "jcbb/Hypothesis.h"

#include "ml/MaximumLikelihood.h"
#include "gt/KnownDataAssociation.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

#include <iostream>
#include <chrono>

namespace slam
{

  template<class POSE, class POINT>
  SLAM<POSE, POINT>::SLAM()
      : latest_pose_key_(0),
        latest_landmark_key_(0)
  {
  }

  template<class POSE, class POINT>
  void SLAM<POSE, POINT>::initialize(double ic_prob, int optimization_rate, const gtsam::Vector &pose_prior_noise, const gtsam::Vector &lmk_prior_noise)
  {
  pose_prior_noise_ = pose_prior_noise;
    lmk_prior_noise_ = lmk_prior_noise;
    ic_prob_ = ic_prob;
    optimization_rate_ = optimization_rate;

    // for (int i = 0; i < 6; i++)
    // {
    //   if (i < 3)
    //   {
    //     odom_sigmas(i) = odom_noise[i] * M_PI / 180.0;
    //     meas_sigmas(i) = meas_noise[i] * M_PI / 180.0;
    //     prior_sigmas(i) = prior_noise[i] * M_PI / 180.0;
    //   }
    //   else
    //   {
    //     odom_sigmas(i) = odom_noise[i];
    //     meas_sigmas(i) = meas_noise[i];
    //     prior_sigmas(i) = prior_noise[i];
    //   }
    // }
    // std::cout << "Using ic_prob " << ic_prob << ", jc_prob " << jc_prob << ", optimization_rate " << optimization_rate << "\n";
    // std::cout << "Using odom sigmas\n"
    //           << odom_sigmas << "\nmeas sigmas\n"
    //           << meas_sigmas << "\nprior sigmas\n"
    //           << prior_sigmas << "\n";

    // Add prior on first pose
    graph_.add(gtsam::PriorFactor<POSE>(X(latest_pose_key_), POSE(), pose_prior_noise_));
    estimates_.insert(X(latest_pose_key_), POSE());

  //   association_method_ = association_method;
  //   std::cout << "Using association method ";
  //   switch (association_method)
  //   {
  //   case AssociationMethod::JCBB:
  //   {
  //     std::cout << "JCBB\n";
  //     break;
  //   }
  //   case AssociationMethod::ML:
  //   {
  //     std::cout << "ML\n";
  //     break;
  //   }
  //   case AssociationMethod::KnownDataAssociation:
  //   {
  //     std::cout << "Known data association\n";
  //     break;
  //   }
  //   }
  }

  template<class POSE, class POINT>
  gtsam::FastVector<POSE> SLAM<POSE, POINT>::getTrajectory() const
  {
    gtsam::FastVector<POSE> trajectory;
    for (int i = 0; i < latest_pose_key_; i++)
    {
      trajectory.push_back(estimates_.at<POSE>(X(i)));
    }
    return trajectory;
  }

  template<class POSE, class POINT>
  gtsam::FastVector<POINT> SLAM<POSE, POINT>::getLandmarkPoints() const
  {
    gtsam::FastVector<POINT> landmarks;
    for (int i = 0; i < latest_landmark_key_; i++)
    {
      landmarks.push_back(estimates_.at<POINT>(L(i)));
    }
    return landmarks;
  }

  template<class POSE, class POINT>
  void SLAM<POSE, POINT>::processOdomMeasurementScan(const Odometry<POSE> &odom, const gtsam::FastVector<Measurement<POINT>> &measurements)
  {
    static int num = 1;
    addOdom(odom);
    if (num % optimization_rate_ != 0)
    {
      num++;
      return;
    }
    num = 1;

    gtsam::Marginals marginals = gtsam::Marginals(graph_, estimates_);

    jcbb::Hypothesis h = jcbb::Hypothesis::empty_hypothesis();
    // switch (association_method_)
    // {
    // case AssociationMethod::JCBB:
    // {
    //   jcbb::JCBB jcbb_(estimates_, marginals, measurements, meas_noise_, ic_prob_, jc_prob_);
    //   h = jcbb_.associate();
    //   break;
    // }
    // case AssociationMethod::ML:
    // {
    //   ml::MaximumLikelihood ml_(estimates_, marginals, measurements, meas_noise_, ic_prob_);
    //   h = ml_.associate();
    //   break;
    // }
    // case AssociationMethod::KnownDataAssociation:
    //   gt::KnownDataAssociation gt_(
    //     estimates_, 
    //     marginals, 
    //     measurements,
    //     meas_noise_,
    //     measured_apriltags,
    //     &apriltag_id_lmk_
    //   );
    //   h = gt_.associate();
    // }

      ml::MaximumLikelihood ml_(estimates_, marginals, measurements, ic_prob_);
      h = ml_.associate();

    const auto &assos = h.associations();
    POSE T_wb = estimates_.at<POSE>(X(latest_pose_key_));
    for (int i = 0; i < assos.size(); i++)
    {
      jcbb::Association::shared_ptr a = assos[i];
      POINT meas = measurements[a->measurement].measurement;
      const auto& meas_noise = measurements[a->measurement].noise;
      POINT meas_world = T_wb * meas;
      if (a->associated())
      {
        // std::cout << "associated meas " << a->measurement << " with landmark " << gtsam::symbolIndex(*a->landmark) << "\n";
        // if ((apriltag_id_lmk_.find(apriltag_id) != apriltag_id_lmk_.end()) && (apriltag_id_lmk_[apriltag_id] != *a->landmark)) {
        //   std::cout << "Associated apriltag " << apriltag_id
        //   << " that was previously associated with landmark " << gtsam::symbolIndex(apriltag_id_lmk_[apriltag_id])
        //   << " with different landmark " << gtsam::symbolIndex(*a->landmark)
        //   << "!!!\n";
        // }
        graph_.add(gtsam::PoseToPointFactor<POSE, POINT>(X(latest_pose_key_), *a->landmark, meas, meas_noise));
      }
      else
      {
        graph_.add(gtsam::PoseToPointFactor<POSE, POINT>(X(latest_pose_key_), L(latest_landmark_key_), meas, meas_noise));
        estimates_.insert(L(latest_landmark_key_), meas_world);
        // std::cout << "Added " << gtsam::symbolChr(L(latest_landmark_key_)) << gtsam::symbolIndex(L(latest_landmark_key_)) << "\n";
        // apriltag_lmk_assos_[apriltag_id].push_back(L(latest_landmark_key_));
        // // Only do this if we are checking how well we correctly associate landmarks
        // if (association_method_ != AssociationMethod::KnownDataAssociation) {
        //   // New apriltag, correctly detected that this haven't been seen before
        //   if (apriltag_id_lmk_.find(apriltag_id) == apriltag_id_lmk_.end())
        //     std::cout << "Associated new landmark " << gtsam::symbolIndex(L(latest_landmark_key_)) << " with apriltag " << apriltag_id << "\n";
        //   apriltag_id_lmk_.insert({apriltag_id, L(latest_landmark_key_)});
        // }
        incrementLatestLandmarkKey();
      }
      // We just inserted our first landmark, set prior
      if (latest_landmark_key_ == 1)
      {
        graph_.add(gtsam::PriorFactor<POINT>(L(0), meas_world, lmk_prior_noise_));
      }
    }
    if (h.num_associations() > 0)
    {
      hypotheses_.push_back(h);
    }

    gtsam::DoglegParams params;
    params.setAbsoluteErrorTol(1e-08);
    estimates_ =
        gtsam::DoglegOptimizer(graph_, estimates_, params).optimize();
  }

  template<class POSE, class POINT>
  void SLAM<POSE, POINT>::addOdom(const Odometry<POSE> &odom)
  {
    graph_.add(gtsam::BetweenFactor<POSE>(X(latest_pose_key_), X(latest_pose_key_ + 1), odom.odom, odom.noise));
    POSE this_pose = latest_pose_ * odom.odom;
    estimates_.insert(X(latest_pose_key_ + 1), this_pose);
    latest_pose_ = this_pose;
    incrementLatestPoseKey();
  }

  template<class POSE, class POINT>
  gtsam::FastVector<POINT> SLAM<POSE, POINT>::predictLandmarks() const
  {
    gtsam::KeyList landmark_keys = estimates_.filter(gtsam::Symbol::ChrTest('l')).keys();
    if (landmark_keys.size() == 0)
    {
      return {};
    }
    gtsam::FastVector<POINT> predicted_measurements;
    for (const auto &lmk : landmark_keys)
    {
      predicted_measurements.push_back(estimates_.at<POINT>(lmk));
    }

    return predicted_measurements;
  }
} // namespace slam