#include <cmath>
#include <eigen3/Eigen/Core>
#include "jcbb/utils.h"

namespace jcbb
{

bool isSymmetric(const Eigen::MatrixXd &m, const double prec)
{
    return ((m - m.transpose()).array().abs() < prec).all();
}

} // namespace jcbb
