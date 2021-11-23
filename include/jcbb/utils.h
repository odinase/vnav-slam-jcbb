#ifndef UTILS_H
#define UTILS_H

#include <eigen3/Eigen/Core>

namespace jcbb
{
    constexpr double wrapToPi(double angle)
    {
        return fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }
    bool isSymmetric(const Eigen::MatrixXd &m, const double prec = 1e-3);
} // namespace jcbb

#endif // UTILS_H