#ifndef MARGINAL_MOCKS_H
#define MARGINAL_MOCKS_H

#include <eigen3/Eigen/Core>
#include <gtsam/nonlinear/Marginals.h>

namespace jcbb
{

class JointMarginalMock
{
public:
    explicit JointMarginalMock(const Eigen::MatrixXd &P, const gtsam::KeyVector& keys);
    const Eigen::Block<const gtsam::Matrix> operator()(gtsam::Key iVariable, gtsam::Key jVariable) const;
    const Eigen::Block<const gtsam::Matrix> at(gtsam::Key iVariable, gtsam::Key jVariable) const { return (*this)(iVariable, jVariable); }
    const gtsam::Matrix& fullMatrix() const {return P_joint_;}

private:
    gtsam::Matrix P_joint_;
    gtsam::KeyVector keys_;
};

class MarginalsMock
{
public:
    explicit MarginalsMock(const Eigen::MatrixXd &P) : P_full_(P) {}
    const Eigen::Block<const gtsam::Matrix> marginalCovariance(gtsam::Key variable) const;
    JointMarginalMock jointMarginalCovariance(const gtsam::KeyVector &variables) const;

private:
    Eigen::MatrixXd P_full_;
};

} // namespace jcbb

#endif // MARGINAL_MOCKS_H