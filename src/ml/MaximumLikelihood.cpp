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
    for (int i = 0; i < measurements_.size(); i++)
    {
      const auto &meas = measurements_[i];

      double lowest_nis = std::numeric_limits<double>::infinity();

      std::pair<int, double> smallest_innovation(-1, 0.0);
      for (const auto &l : landmark_keys_)
      {
        gtsam::Pose3 lmk = estimates_.at<gtsam::Pose3>(l);
        gtsam::BetweenFactor<gtsam::Pose3> factor(x_key_, l, meas, noise);
        gtsam::Vector error = factor.evaluateError(x_pose_, lmk);
        Association a(i, l, error);
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
            smallest_innovation = {l, nis};
          }
        }
      }
      // We found a valid association
      if (smallest_innovation.first != -1)
      {
        lmk_measurement_assos[smallest_innovation.first].push_back({i, smallest_innovation.second});
      }
    }

    // Loop over all landmarks with measurement associated with it and pick put the best one by pruning out all measurements except the best one in terms of Mahalanobis distance.
    Hypothesis h = Hypothesis::empty_hypothesis();
    for (const auto &[l, ms] : lmk_measurement_assos)
    {
      auto p = std::min_element(ms.begin(), ms.end(), [](const auto &p1, const auto &p2)
                                { return p1.second < p2.second; });
      Association::shared_ptr a{std::make_shared<Association>(l, p->first, p->second)};
      h.extend(a);
    }
    return h;
  }
} // namespace ml
