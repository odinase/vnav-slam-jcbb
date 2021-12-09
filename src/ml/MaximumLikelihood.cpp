#include "ml/MaximumLikelihood.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <iostream>
#include "jcbb/jcbb.h"
#include <utility>
#include <algorithm>
#include <memory>

namespace ml
{
  using gtsam::symbol_shorthand::L;
  using gtsam::symbol_shorthand::X;

  MaximumLikelihood::MaximumLikelihood(const gtsam::Values &estimates, const gtsam::Marginals &marginals, const gtsam::FastVector<gtsam::Pose3> &measurements, const gtsam::noiseModel::Diagonal::shared_ptr &meas_noise, double ic_prob)
      : estimates_(estimates),
        marginals_(marginals),
        measurements_(measurements),
        meas_noise_(meas_noise),
        ic_prob_(ic_prob)
  {
    landmark_keys_ = estimates_.filter(gtsam::Symbol::ChrTest('l')).keys();
    auto poses = estimates_.filter(gtsam::Symbol::ChrTest('x'));
    int last_pose = poses.size() - 1; // Assuming first pose is 0
    x_key_ = X(last_pose);
    x_pose_ = estimates.at<gtsam::Pose3>(x_key_);
  }

  double MaximumLikelihood::joint_compatability(const Hypothesis &h) const
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

  double MaximumLikelihood::individual_compatability(const Association &a) const
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

  Hypothesis MaximumLikelihood::associate() const
  {
    gtsam::SharedNoiseModel noise(meas_noise_);
    // First loop over all measurements, and find the lowest Mahalanobis distance
    gtsam::FastMap<gtsam::Key, gtsam::FastVector<std::pair<int, double>>> lmk_measurement_assos;
    gtsam::Matrix Hx, Hl;
    for (int i = 0; i < measurements_.size(); i++)
    {
      const auto &meas = measurements_[i];

      double lowest_nis = std::numeric_limits<double>::infinity();

      std::pair<int, double> smallest_innovation(-1, 0.0);
      for (const auto &l : landmark_keys_)
      {
        gtsam::Pose3 lmk = estimates_.at<gtsam::Pose3>(l);
        gtsam::BetweenFactor<gtsam::Pose3> factor(x_key_, l, meas, noise);
        gtsam::Vector error = factor.evaluateError(x_pose_, lmk, Hx, Hl);
        Association a(i, l, Hx, Hl, error);
        double nis = individual_compatability(a);

        // TODO: Refactor out things not JCBB from jcbb
        double inv = jcbb::chi2inv(1 - ic_prob_, gtsam::Pose3::dimension);
        // Individually compatible?
        if (nis < inv)
        {
          // Better association than already found?
          if (nis < lowest_nis)
          {
            lowest_nis = nis;
            smallest_innovation = {gtsam::symbolIndex(l), nis};
          }
        }
      }
      // We found a valid association
      if (smallest_innovation.first != -1)
      {
        lmk_measurement_assos[L(smallest_innovation.first)].push_back({i, smallest_innovation.second});
      }
    }

    // Loop over all landmarks with measurement associated with it and pick put the best one by pruning out all measurements except the best one in terms of Mahalanobis distance.
    Hypothesis h = Hypothesis::empty_hypothesis();
    for (const auto &[l, ms] : lmk_measurement_assos)
    {
      auto p = std::min_element(ms.begin(), ms.end(), [](const auto &p1, const auto &p2)
                                { return p1.second < p2.second; });
      // Pretty redundant to do full recomputation here, but oh well
      gtsam::Pose3 lmk = estimates_.at<gtsam::Pose3>(l);
      const auto &meas = measurements_[p->first];
      gtsam::BetweenFactor<gtsam::Pose3> factor(x_key_, l, meas, noise);
      gtsam::Vector error = factor.evaluateError(x_pose_, lmk, Hx, Hl);
      Association::shared_ptr a = std::make_shared<Association>(p->first, l, Hx, Hl, error);
      h.extend(a);
    }
    h.fill_with_unassociated_measurements(measurements_.size());

    double nis = joint_compatability(h);
    h.set_nis(nis);

    return h;
  }
} // namespace ml
