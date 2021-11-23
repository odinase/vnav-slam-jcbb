#ifndef BEARING_RANGE_FACTOR_H
#define BEARING_RANGE_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <cmath>
#include <eigen3/Eigen/Core>

#include "jcbb/jcbb.h"
#include "jcbb/utils.h"


namespace jcbb
{
    class RangeBearingFactor : public gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Point2>
    {
    private:
        double range_;
        double bearing_;
        Eigen::MatrixXd sensorOffset_;

    public:
        RangeBearingFactor(const gtsam::Key &ki, const gtsam::Key &kj, double range, double bearing, const Eigen::MatrixXd& sensorOffset, gtsam::SharedNoiseModel &model)
            : gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Point2>(model, ki, kj),
              bearing_(bearing),
              range_(range),
              sensorOffset_(sensorOffset) {}

        gtsam::Vector evaluateError(const gtsam::Pose2 &x, const gtsam::Point2 &lmk, boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2) const
        {
            gtsam::Vector error(2);
            gtsam::Rot2 Rr = x.rotation();
            gtsam::Vector dp = Rr.inverse() * (lmk - x.translation()) - sensorOffset_;
            double range = dp.norm();
            double bearing = atan2(dp(1), dp(0));
            error << range_ - range, wrapToPi(bearing_ - bearing);
            if (H1)
            {
                gtsam::Vector n = Rr * dp;
                *H1 = gtsam::Matrix(2, 3);
                double n_norm = n.norm();
                gtsam::Rot2 R(M_PI / 2.0);
                gtsam::Matrix jac_z_b(2, 3);
                jac_z_b << -gtsam::Matrix::Identity(2,2), -R.matrix()*(lmk - x.translation());
                *H1 << n.transpose() / n_norm*jac_z_b,
                    n.transpose() / (n_norm * n_norm) * R.matrix().transpose()*jac_z_b;
            }
            if (H2)
            {
                gtsam::Vector n = Rr* dp;
                *H2 = gtsam::Matrix(2, 2);
                double n_norm = n.norm();
                gtsam::Rot2 R(M_PI / 2.0);
                *H2 << n_norm * n.transpose(),
                    n.transpose() * R.matrix().transpose();
                *H2 = *H2 / (n_norm * n_norm);
            }
            return error;
        }
    };

} // namespace jcbb


#endif // BEARING_RANGE_FACTOR_H