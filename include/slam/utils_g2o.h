/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file landmarkSLAM_g2o.cpp
 * @brief A 2D/3D landmark SLAM pipeline that reads input from g2o, converts it
 * to a factor graph and does the optimization. Output is written on a file, in
 * g2o format Syntax for the script is ./landmarkSLAM_g2o input.g2o is3D
 * output.g2o
 * @date December 15, 2021
 * @author Luca Carlone, Yihao Zhang
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/slam/PoseToPointFactor.h>

#include <cmath>
#include <fstream>
#include <boost/filesystem/path.hpp>
#include <ctime>
using gtsam::symbol_shorthand::L;  // gtsam/slam/dataset.cpp

namespace gtsam {

#define LINESIZE 81920  // dataset.cpp. ignore lines after this

// from gtsam4.0.3/slam/dataset.cpp
static Rot3 NormalizedRot3(double w, double x, double y, double z) {
  const double norm = sqrt(w * w + x * x + y * y + z * z), f = 1.0 / norm;
  return Rot3::Quaternion(f * w, f * x, f * y, f * z);
}

void saveException(const std::string &g2ofilepath, const std::string &algName,
                   const char* err_msg, const std::string &other_msg) {
  auto g2opath = boost::filesystem::path(g2ofilepath);
  auto fname = boost::filesystem::path(algName);
  auto savepath = g2opath.parent_path() / fname;
  std::fstream stream(savepath.c_str(), std::fstream::app);
  stream << "\nException: ";
  stream << err_msg << std::endl;
  stream << other_msg << std::endl;
  stream << g2opath.filename() << std::endl;
  stream.close();
}

// save graph errors in the same directory as the g2o file
void saveGraphErrors(const std::string &g2ofilepath, const std::string &algName,
                     const std::vector<double> &errors) {
  auto g2opath = boost::filesystem::path(g2ofilepath);
  auto fname = boost::filesystem::path(
    std::string("errors_graph_") + algName + std::string(".txt"));
  auto errorSavePath = g2opath.parent_path() / fname; // concat
  auto g2ofilename = g2opath.filename();
  // size_t lastindex = g2ofilename.find_last_of("."); 
  // string nameonly = g2ofilename.substr(0, lastindex); // remove extension
  std::fstream stream(errorSavePath.c_str(), std::fstream::app); // append mode ::out overwrites
  std::time_t now = std::time(0);
  char* dt = ctime(&now);
  stream << dt << g2ofilename.c_str() << std::endl;
  for (int i = 0; i < errors.size(); i++) {
    stream << errors[i];
    if (i != errors.size() - 1) {
      stream << ", ";
    }
  }
  stream << std::endl;
  stream.close();
}

void saveVector(const std::string &g2ofilepath, const std::string &saveName,
                const std::vector<double> &errors) {
  auto g2opath = boost::filesystem::path(g2ofilepath);
  auto fname = boost::filesystem::path(saveName);
  auto savepath = g2opath.parent_path() / fname;
  std::fstream stream(savepath.c_str(), std::fstream::out); // write mode overwrites
  for (int i = 0; i < errors.size(); i++) {
    stream << errors[i];
    if (i != errors.size() - 1) {
      stream << ", ";
    }
  }
  stream.close();
}

std::vector<size_t> findPoseToPointFactors(
    std::vector<boost::shared_ptr<PoseToPointFactor<Pose2, Point2>>> &lmFactors2d,
    std::vector<boost::shared_ptr<PoseToPointFactor<Pose3, Point3>>> &lmFactors3d,
    const NonlinearFactorGraph::shared_ptr &graph) {
  assert(lmFactors2d.size() == 0 && lmFactors3d.size() == 0);
  boost::shared_ptr<PoseToPointFactor<Pose2, Point2>> factor2d;
  boost::shared_ptr<PoseToPointFactor<Pose3, Point3>> factor3d;
  size_t factor_idx = 0;
  std::vector<size_t> lmFactorIdx;
  for (const auto &factor_ : *graph) { // inspired by dataset.cpp/writeG2o
    factor3d = boost::dynamic_pointer_cast<PoseToPointFactor<Pose3, Point3>>(factor_);
    factor2d = boost::dynamic_pointer_cast<PoseToPointFactor<Pose2, Point2>>(factor_);
    if (factor2d || factor3d) {
      lmFactorIdx.push_back(factor_idx);
    }
    if (factor2d) {
      lmFactors2d.push_back(factor2d);
    }
    if (factor3d) {
      lmFactors3d.push_back(factor3d);
    }
    factor_idx++;
  }
  return lmFactorIdx;
}


std::pair<std::vector<size_t>, std::vector<size_t>> findFactors(
    std::vector<boost::shared_ptr<BetweenFactor<Pose2>>> &odomFactors2d,
    std::vector<boost::shared_ptr<BetweenFactor<Pose3>>> &odomFactors3d,
    std::vector<boost::shared_ptr<PoseToPointFactor<Pose2, Point2>>> &measFactors2d,
    std::vector<boost::shared_ptr<PoseToPointFactor<Pose3, Point3>>> &measFactors3d,
    const NonlinearFactorGraph::shared_ptr &graph) {
  assert(odomFactors2d.size() == 0 && odomFactors3d.size() == 0);
  boost::shared_ptr<PoseToPointFactor<Pose2, Point2>> measFactor2d;
  boost::shared_ptr<PoseToPointFactor<Pose3, Point3>> measFactor3d;
  boost::shared_ptr<BetweenFactor<Pose2>> odomFactor2d;
  boost::shared_ptr<BetweenFactor<Pose3>> odomFactor3d;
  size_t factor_idx = 0;
  std::vector<size_t> measFactorIdx;
  std::vector<size_t> odomFactorIdx;
  for (const auto &factor_ : *graph) { // inspired by dataset.cpp/writeG2o
    measFactor2d = boost::dynamic_pointer_cast<PoseToPointFactor<Pose2, Point2>>(factor_);
    measFactor3d = boost::dynamic_pointer_cast<PoseToPointFactor<Pose3, Point3>>(factor_);
    odomFactor2d = boost::dynamic_pointer_cast<BetweenFactor<Pose2>>(factor_);
    odomFactor3d = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor_);
    if (measFactor2d || measFactor3d) {
      measFactorIdx.push_back(factor_idx);
      if (measFactor2d) {
        measFactors2d.push_back(measFactor2d);
      }
      if (measFactor3d) {
        measFactors3d.push_back(measFactor3d);
      }
    }
    if (odomFactor2d || odomFactor3d) {
      odomFactorIdx.push_back(factor_idx);
      if (odomFactor2d) {
        odomFactors2d.push_back(odomFactor2d);
      }
      if (odomFactor3d) {
        odomFactors3d.push_back(odomFactor3d);
      }
    }
    
    factor_idx++;
  }
  return std::make_pair(odomFactorIdx, measFactorIdx);
}


KeyVector findLmKeys(const Values::shared_ptr &initial) {
  KeyVector ldmk_keys;
  for (const auto key_value : initial->filter(Symbol::ChrTest('l'))) {
    ldmk_keys.push_back(key_value.key);
    //cout << DefaultKeyFormatter(key_value.key) << std::endl;
  }
  return ldmk_keys;
}

KeyVector findPoseKeys(const Values::shared_ptr &initial, const KeyVector &ldmk_keys) {
  KeyVector pose_keys;
  for (const auto key_value : *initial) {
    if (std::find(ldmk_keys.begin(), ldmk_keys.end(), key_value.key) == ldmk_keys.end()) {
      pose_keys.push_back(key_value.key);
    }
  }
  return pose_keys;
}

// may read landmark measurements if tagged by 'BR' or 'LANDMARK' (see dataset.cpp)
// TODO(LC): what about potential loop closures?
std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> readG2oOdomOnly(
    const std::string &g2oFile, const bool is3D, const std::string &kernelType) {
  NonlinearFactorGraph::shared_ptr odom_graph;
  Values::shared_ptr initial;

  if (kernelType.compare("none") == 0) {
    boost::tie(odom_graph, initial) =
        readG2o(g2oFile, is3D);  // load 2d: 2d pose, 2d ldmk point2
  }
  if (kernelType.compare("huber") == 0) {
    std::cout << "Using robust kernel: huber " << std::endl;
    boost::tie(odom_graph, initial) =
        readG2o(g2oFile, is3D, KernelFunctionTypeHUBER);
  }
  if (kernelType.compare("tukey") == 0) {
    std::cout << "Using robust kernel: tukey " << std::endl;
    boost::tie(odom_graph, initial) =
        readG2o(g2oFile, is3D, KernelFunctionTypeTUKEY);
  }

  // Add prior on the pose having index (key) = 0
  if (is3D) {
    auto priorModel = noiseModel::Diagonal::Variances(
        (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    odom_graph->addPrior(0, Pose3(), priorModel);
  } else {
    auto priorModel =  //
        noiseModel::Diagonal::Variances(Vector3(1e-6, 1e-6, 1e-8));
    odom_graph->addPrior(0, Pose2(), priorModel);
  }
  std::cout << "Adding prior on pose 0." << std::endl;

  return std::make_pair(odom_graph, initial);
}

std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> readG2owithLmks(
    const std::string &g2oFile, const bool is3D, const std::string &kernelType) {
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  
  boost::tie(graph, initial) = readG2oOdomOnly(g2oFile, is3D, kernelType);  

  // reading file for landmark estimates and landmark edges (gtsam4.0.3
  // slam/dataset.cpp)
  std::ifstream is(g2oFile.c_str());
  if (!is) throw std::invalid_argument("cannot find file " + g2oFile);
  std::string tag;

  Key id1, id2;
  while (!is.eof()) {
    if (!(is >> tag)) break;

    // add 2d ldmk pose2point factor (if any)
    if (tag == "EDGE_SE2_XY") {
      double lmx, lmy;
      double v1, v2, v3;

      is >> id1 >> id2 >> lmx >> lmy >> v1 >> v2 >> v3;

      // Convert to cov (assuming diagonal mat)
      // v1 = 1.0 / v1;
      // v3 = 1.0 / v3;
      // Create noise model
      // noiseModel::Diagonal::shared_ptr measurementNoise =
      //     noiseModel::Diagonal::Variances((Vector(2) << v1, v3).finished());

      // Create noise model
      Matrix2 info_mat;
      info_mat << v1, v2, v2, v3;
      noiseModel::Gaussian::shared_ptr measurementNoise =
          noiseModel::Gaussian::Information(info_mat, true); // smart = true

      // Add to graph
      *graph += gtsam::PoseToPointFactor<Pose2, Point2>(
          id1, L(id2), Point2(lmx, lmy), measurementNoise);
    }

    // add 3d ldmk pose2point factor (if any)
    if (tag == "EDGE_SE3_XYZ") {
      double lmx, lmy, lmz;
      double v11, v12, v13, v22, v23, v33;

      is >> id1 >> id2 >> lmx >> lmy >> lmz >> v11 >> v12 >> v13 >> v22 >>
          v23 >> v33;

      // Convert to cov (assuming diagonal mat)
      // v11 = 1.0 / v11;
      // v22 = 1.0 / v22;
      // v33 = 1.0 / v33;
      // Create noise model
      // noiseModel::Diagonal::shared_ptr measurementNoise =
      //     noiseModel::Diagonal::Variances(
      //         (Vector(3) << v11, v22, v33).finished());

      // Create noise model
      Matrix3 info_mat;
      info_mat << v11, v12, v13, v12, v22, v23, v13, v23, v33;
      noiseModel::Gaussian::shared_ptr measurementNoise =
          noiseModel::Gaussian::Information(info_mat, true);

      // Add to graph
      *graph += gtsam::PoseToPointFactor<Pose3, Point3>(
          id1, L(id2), Point3(lmx, lmy, lmz), measurementNoise);
    }
    is.ignore(LINESIZE, '\n');
  }
  is.clear();
  is.seekg(0, std::ios::beg);  // guess back to beginning

  return std::make_pair(graph, initial);
}

template <class POINT>
bool isPutativeAssociation(const Key key1, const Key key2, const Vector diff,
                           const gtsam::Marginals &marginals) {
  // Current joint marginals of the 2 landmarks
  gtsam::KeyVector keys;
  keys.push_back(key1);
  keys.push_back(key2);

  // Compute covariance of difference
  Matrix covJoint = marginals.jointMarginalCovariance(keys).fullMatrix();
  size_t varSize = covJoint.rows() / 2;
  Matrix diffMat =
      (Matrix(varSize, 2 * varSize) << Matrix::Identity(varSize, varSize),
       -Matrix::Identity(varSize, varSize))
          .finished();
  Matrix covDiff = diffMat * covJoint * diffMat.transpose();
  Matrix mahDist = diff.transpose() * covDiff.inverse() * diff;
  return mahDist(0, 0) < 9.0;  // ~chi2inv(0.99,2)
}

}  // namespace gtsam
