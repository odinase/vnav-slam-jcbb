#include "MarginalMocks.h"
#include <eigen3/Eigen/Core>
#include <gtsam/inference/Symbol.h>
#include <iostream>

namespace jcbb
{
  JointMarginalMock::JointMarginalMock(const Eigen::MatrixXd &P, const gtsam::KeyVector &keys) : P_joint_(P), keys_(keys) {}

  const Eigen::Block<const gtsam::Matrix> JointMarginalMock::operator()(gtsam::Key iVariable, gtsam::Key jVariable) const
  {
    int index_i, index_j, i, j, di, dj, offset_i, offset_j;
    if (gtsam::symbolChr(iVariable) == 'x') {
      i = 0;
      di = 3;
      offset_i = 0;
      index_i = 0;
    } else {
      auto p = std::find(keys_.begin(), keys_.end(), iVariable);
      i = std::distance(++keys_.begin(), p);
      di = 2;
      offset_i = 3;
      index_i = 3 + di*i;
    }
    if (gtsam::symbolChr(jVariable) == 'x') {
      j = 0;
      dj = 3;
      offset_j = 0;
      index_j = 0;
    } else {
      auto p = std::find(keys_.begin(), keys_.end(), jVariable);
      j = std::distance(++keys_.begin(), p);
      dj = 2;
      offset_j = 3;
      index_j = 3 + dj*j;
    }
    return P_joint_.block(index_i, index_j, di, dj);
  }

  const Eigen::Block<const gtsam::Matrix> MarginalsMock::marginalCovariance(gtsam::Key variable) const
  {
    return P_full_.block(0, 0, 1, 1);
  }

  JointMarginalMock MarginalsMock::jointMarginalCovariance(const gtsam::KeyVector &variables) const
  {
    const int dim = 2 * variables.size() + 1; // Assumes first key is robot pose and rest landmarks
    gtsam::Matrix P_joint(dim, dim);

    // Insert state row on top
    P_joint.block<3, 3>(0, 0) = P_full_.block<3, 3>(0, 0);
    int i = 1, j = 3;
    for (int i = 1; i < variables.size(); i++)
    {
      int lj = gtsam::symbolIndex(variables[i]);
      P_joint.block<3, 2>(0, j) = P_full_.block<3, 2>(0, 3 + 2 * lj); // Assumes landmarks are zero indexed
      j += 2;
    }

    // Insert landmarks
    for (int i = 1; i < variables.size(); i++)
    {
      int li = gtsam::symbolIndex(variables[i]);
      for (int j = 1; j < variables.size(); j++)
      {
        int lj = gtsam::symbolIndex(variables[j]);
        P_joint.block<2, 2>(3 + 2 * (i - 1), 3 + 2 * (j - 1)) = P_full_.block<2, 2>(3 + 2 * li, 3 + 2 * lj);
      }
    }
    P_joint.triangularView<Eigen::Lower>() = P_joint.transpose();

    return JointMarginalMock(P_joint, variables);
  }

} // namespace jcbb