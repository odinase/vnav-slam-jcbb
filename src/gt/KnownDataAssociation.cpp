#include "gt/KnownDataAssociation.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <iostream>
#include "jcbb/jcbb.h"
#include <utility>
#include <algorithm>
#include <memory>

namespace gt
{
  using gtsam::symbol_shorthand::L;
  using gtsam::symbol_shorthand::X;

  KnownDataAssociation::KnownDataAssociation(
    const gtsam::Values &estimates,
    const gtsam::Marginals &marginals,
    const gtsam::FastVector<gtsam::Pose3> &measurements,
    const gtsam::noiseModel::Diagonal::shared_ptr &meas_noise,
    const gtsam::FastVector<int> &measured_apriltags,
    std::unordered_map<int, gtsam::Key>* apriltag_id_lmk
  )
      : estimates_(estimates),
        marginals_(marginals),
        measurements_(measurements),
        meas_noise_(meas_noise),
        measured_apriltags_(measured_apriltags),
        apriltag_id_lmk_(apriltag_id_lmk)
  {
    auto lmk_keys = estimates_.filter(gtsam::Symbol::ChrTest('l'));
    lmk_key_ = lmk_keys.size(); // If we have added n landmarks, the next is index n (0, ..., n-1)

    auto poses = estimates_.filter(gtsam::Symbol::ChrTest('x'));
    int last_pose = poses.size() - 1; // Assuming first pose is 0

    x_key_ = X(last_pose);
    x_pose_ = estimates.at<gtsam::Pose3>(x_key_);
  }

  double KnownDataAssociation::joint_compatability(const Hypothesis &h) const
  {
    int N = h.num_associations();
    int n = gtsam::Pose3::dimension;
    int m = gtsam::Pose3::dimension;
    int d = gtsam::Pose3::dimension;

    gtsam::KeyVector joint_states;
    joint_states.push_back(x_key_);
    int num_associated_meas_to_lmk = 0;
    for (const auto &asso : h.associations())
    {
      if (asso->associated())
      {
        num_associated_meas_to_lmk++;
        joint_states.push_back(*asso->landmark);
      }
    }

    Eigen::MatrixXd Pjoint = marginals_.jointMarginalCovariance(joint_states).fullMatrix();

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(num_associated_meas_to_lmk * d, n + num_associated_meas_to_lmk * m);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(num_associated_meas_to_lmk * d, num_associated_meas_to_lmk * d);

    Eigen::VectorXd innov(N * d);
    int k = 0, j = 0;

    for (const auto &a : h.associations())
    {
      if (a->associated())
      {
        innov.segment(k, d) = a->error;
        H.block(k, 0, d, n) = a->Hx;
        H.block(k, n + j, d, m) = a->Hl;

        // Adding R might be done more cleverly
        R.block(k, k, d, d) = meas_noise_->sigmas().array().square().matrix().asDiagonal();

        k += d;
        j += m;
      }
    }

    Eigen::MatrixXd Sjoint = H * Pjoint * H.transpose() + R;

    double nis = innov.transpose() * Sjoint.llt().solve(innov);
    return nis;
  }

  double KnownDataAssociation::individual_compatability(const Association &a) const
  {
    // Should never happen...
    if (!a.associated())
    {
      return std::numeric_limits<double>::infinity();
    }
    Eigen::VectorXd innov = a.error;
    Eigen::MatrixXd P = marginals_.jointMarginalCovariance(gtsam::KeyVector{{x_key_, *a.landmark}}).fullMatrix();
    // TODO: Fix here later
    int rows = a.Hx.rows();
    int cols = a.Hx.cols() + a.Hl.cols();
    Eigen::MatrixXd H(rows, cols);
    H << a.Hx, a.Hl;
    Eigen::MatrixXd R = meas_noise_->sigmas().array().square().matrix().asDiagonal();
    Eigen::MatrixXd S = H * P * H.transpose() + R;

    return innov.transpose() * S.llt().solve(innov);
  }

  Hypothesis KnownDataAssociation::associate()
  {
    gtsam::SharedNoiseModel noise(meas_noise_);
    gtsam::Matrix Hx, Hl;

    Hypothesis h = Hypothesis::empty_hypothesis();
    // Loop over all detected measurements
    for (int i = 0; i < measurements_.size(); i++)
    {
      int apriltag_id = measured_apriltags_[i];
      const auto &meas = measurements_[i];
      // Seen before?
      Association::shared_ptr a;

      if (apriltag_id_lmk_->find(apriltag_id) != apriltag_id_lmk_->end()) {
        // Associate immediately to landmark
        gtsam::Key l = apriltag_id_lmk_->at(apriltag_id);
        gtsam::Pose3 lmk = estimates_.at<gtsam::Pose3>(l);
        gtsam::BetweenFactor<gtsam::Pose3> factor(x_key_, l, meas, noise);
        gtsam::Vector error = factor.evaluateError(x_pose_, lmk, Hx, Hl);
        a = std::make_shared<Association>(i, l, Hx, Hl, error);
      } else {
        // Create new landmark
        apriltag_id_lmk_->insert({apriltag_id, L(lmk_key_)});
        a = std::make_shared<Association>(i);
        lmk_key_++;
      }
      h.extend(a);
    }

    double nis = joint_compatability(h);
    h.set_nis(nis);

    return h;
  }
} // namespace ml
