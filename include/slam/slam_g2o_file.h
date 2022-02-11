#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/slam/PoseToPointFactor.h>
#include "utils.h"
#include <cmath>
#include <fstream>

class SLAM_G2O_file {
    private:
    std::string g2o_filename_;

    public:
    SLAM_G2O_file(const std::string& filename);
};