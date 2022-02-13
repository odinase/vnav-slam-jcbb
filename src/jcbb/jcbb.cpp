#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <boost/math/distributions.hpp>
#include <cmath>
#include <vector>
#include <memory>
#include <unordered_set>
#include <utility>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <iostream>

#include "jcbb/jcbb.h"
#include "jcbb/utils.h"
// #include "jcbb/bearing_range_factor.h"

namespace jcbb
{
  using gtsam::symbol_shorthand::L;
  using gtsam::symbol_shorthand::X;

  double chi2inv(double p, unsigned int dim)
  {
    boost::math::chi_squared dist(dim);
    return quantile(dist, p);
  }

  bool exists(const gtsam::KeyVector &s, gtsam::Key k)
  {
    return std::find(s.begin(), s.end(), k) != s.end();
  }

  JCBB::JCBB(const gtsam::Values &estimates, const Marginals &marginals, const gtsam::FastVector<gtsam::Pose3> &measurements, const gtsam::noiseModel::Diagonal::shared_ptr &meas_noise, double ic_prob, double jc_prob)
      : estimates_(estimates),
        marginals_(marginals),
        measurements_(measurements),
        meas_noise_(meas_noise),
        ic_prob_(ic_prob),
        jc_prob_(jc_prob)
  {
    landmark_keys_ = estimates_.filter(gtsam::Symbol::ChrTest('l')).keys();
    auto poses = estimates_.filter(gtsam::Symbol::ChrTest('x'));
    int last_pose = poses.size() - 1; // Assuming first pose is 0
    x_key_ = X(last_pose);
    x_pose_ = estimates.at<gtsam::Pose3>(x_key_);
  }

  bool JCBB::prunable(const Hypothesis &h, const Hypothesis &best) const
  {
    int tot_num_measurements = measurements_.size();
    int num_associations = h.num_associations();
    int num_associated_measurements = h.num_measurements();
    int remaining_measurements = tot_num_measurements - num_associated_measurements;
    int potential_num_associations = num_associations + remaining_measurements;

    // If we can't beat the current number of associations, we give up
    return potential_num_associations <= best.num_associations();
  }

  bool JCBB::feasible(const Hypothesis &h) const
  {
    int N = h.num_associations();
    int d = gtsam::Pose3::dimension;
    if (N == 0)
    {
      return true;
    }
    double nis = joint_compatability(h);
    return nis < chi2inv(1 - jc_prob_, N * d);
  }

  void JCBB::push_successors_on_heap(FastMinHeap<Hypothesis> *min_heap, const Hypothesis &h) const
  {
    // Should probably keep this in...
    // if (min_heap == nullptr) {
    //   return;
    // }
    int curr_measurement = h.num_measurements() - 1; // We use zero indexing, so needs to subtract 1
    int next_measurement = curr_measurement + 1;
    int tot_num_measurements = measurements_.size();
    if ((next_measurement + 1) > tot_num_measurements)
    { // We have no more measurements to check, so no successors
      return;
    }

    int d  = gtsam::Pose3::dimension;

    // Add unassociated hypothesis
    Hypothesis successor = h.extended(std::make_shared<Association>(next_measurement));
    min_heap->push(successor);

    gtsam::KeyVector associated_landmarks = h.associated_landmarks();
    gtsam::Pose3 meas = measurements_[next_measurement];
    gtsam::Matrix Hx, Hl;

    gtsam::SharedNoiseModel noise(meas_noise_);

    for (auto &l : landmark_keys_)
    {
      if (exists(associated_landmarks, l))
      {
        continue;
      }
      gtsam::Pose3 lmk = estimates_.at<gtsam::Pose3>(l);

      gtsam::BetweenFactor<gtsam::Pose3> factor(x_key_, l, meas, noise);
      gtsam::Vector error = factor.evaluateError(x_pose_, lmk, Hx, Hl);

      Association a(next_measurement, l, Hx, Hl, error);
      double nis = individual_compatability(a);
      double inv = chi2inv(1 - ic_prob_, d);
      if (nis < inv)
      {
        Hypothesis successor = h.extended(std::make_shared<Association>(a));
        double joint_nis = joint_compatability(successor);
        successor.set_nis(joint_nis);
        min_heap->push(successor);
      }
    }
  }

  double JCBB::joint_compatability(const Hypothesis &h) const
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

  double JCBB::individual_compatability(const Association &a) const
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

  Hypothesis JCBB::associate() const
  {
    Hypothesis best_hypothesis{Hypothesis::empty_hypothesis()};
    FastMinHeap<Hypothesis> min_heap;
    min_heap.push(Hypothesis::empty_hypothesis());

    while (!min_heap.empty())
    {
      Hypothesis hypothesis = min_heap.top();
      min_heap.pop();

      if (!prunable(hypothesis, best_hypothesis))
      {
        if (feasible(hypothesis) && hypothesis.better_than(best_hypothesis))
        {
          best_hypothesis = hypothesis;
        }
        push_successors_on_heap(&min_heap, hypothesis);
      }
    }
    
    // Append non associated measurements at bottom of interpretation tree
    if (best_hypothesis.num_associations() < measurements_.size())
    {
      for (int m = best_hypothesis.num_associations(); m < measurements_.size(); m++)
      {
        best_hypothesis.extend(std::make_shared<Association>(m));
      }
    }

    return best_hypothesis;
  }

} // namespace jcbb
