
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/slam/PoseToPointFactor.h>
// #include "slam/utils.h"
#include <cmath>
#include <fstream>
#include <tuple>
#include <algorithm>

#include <glog/logging.h>

#include "slam/slam_g2o_file.h"
#include "slam/utils_g2o.h"
#include "slam/slam.h"
#include "slam/types.h"

using gtsam::symbol_shorthand::L; // gtsam/slam/dataset.cpp
using namespace std;
using namespace gtsam;

// SLAM_G2O_file::SLAM_G2O_file(const std::string& filename) : g2o_filename_(filename) {
//     boost::tie()
// }

template <class POSE, class POINT>
struct Timestep
{
    int step;
    slam::Odometry<POSE> odom;
    FastVector<slam::Measurement<POINT>> measurements;
};

using Timestep2D = Timestep<Pose2, Point2>;
using Timestep3D = Timestep<Pose3, Point3>;

template <class POSE, class POINT>
std::vector<Timestep<POSE, POINT>> convert_into_timesteps(
    vector<boost::shared_ptr<BetweenFactor<POSE>>> &odomFactors,
    vector<boost::shared_ptr<PoseToPointFactor<POSE, POINT>>> &measFactors)
{
    // Sort factors based on robot pose key, so that we can simply check when in time they should appear
    std::sort(
        odomFactors.begin(),
        odomFactors.end(),
        [](const auto &lhs, const auto &rhs)
        {
            return symbolIndex(lhs->key1()) < symbolIndex(rhs->key1());
        });
    std::sort(
        measFactors.begin(),
        measFactors.end(),
        [](const auto &lhs, const auto &rhs)
        {
            return symbolIndex(lhs->key1()) < symbolIndex(rhs->key1());
        });

    size_t odoms = odomFactors.size();
    uint64_t num_timesteps = odoms + 1; // There will always be one more robot pose than odometry factors since they're all between
    vector<Timestep<POSE, POINT>> timesteps;
    size_t curr_measurement = 0;
    size_t tot_num_measurements = measFactors.size();
    timesteps.reserve(num_timesteps);
    for (uint64_t t = 0; t < num_timesteps; t++)
    {
        Timestep<POSE, POINT> timestep;
        timestep.step = t;
        // Initialize first odom as identity, as we haven't moved yet
        if (t > 0)
        {
            timestep.odom.odom = odomFactors[t - 1]->measured();
            timestep.odom.noise = odomFactors[t - 1]->noiseModel();
        }
        else
        {
            timestep.odom.odom = POSE();
        }

        // Extract measurements from current pose
        while (curr_measurement < tot_num_measurements && symbolIndex(measFactors[curr_measurement]->key1()) == t)
        {
            slam::Measurement<POINT> meas;
            meas.measurement = measFactors[curr_measurement]->measured();
            meas.noise = measFactors[curr_measurement]->noiseModel();
            timestep.measurements.push_back(meas);
            curr_measurement++;
        }

        timesteps.push_back(timestep);
    }

    return timesteps;
}

int main(int argc, char **argv)
{
      google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    // default
    string g2oFile = findExampleDataFile("noisyToyGraph.txt");
    bool is3D = false;

    // Parse user's inputs
    if (argc > 1)
    {
        g2oFile = argv[1]; // input dataset filename
    }
    if (argc > 2)
    {
        is3D = atoi(argv[2]);
        std::cout << "is3D: " << is3D << std::endl;
    }

    vector<boost::shared_ptr<PoseToPointFactor<Pose2, Point2>>> measFactors2d;
    vector<boost::shared_ptr<PoseToPointFactor<Pose3, Point3>>> measFactors3d;

    vector<boost::shared_ptr<BetweenFactor<Pose2>>> odomFactors2d;
    vector<boost::shared_ptr<BetweenFactor<Pose3>>> odomFactors3d;

    // reading file and creating factor graph
    NonlinearFactorGraph::shared_ptr graph;
    Values::shared_ptr initial;
    boost::tie(graph, initial) = readG2owithLmks(g2oFile, is3D, "none");
    auto [odomFactorIdx, measFactorIdx] = findFactors(odomFactors2d, odomFactors3d, measFactors2d, measFactors3d, graph);
    int optimization_rate = 5; // Optimize every n times
    if (is3D)
    {
        double ic_prob = 0.9888910034617577; // chi2.cdf(3**2, 2)
        gtsam::Vector pose_prior_noise = (
            gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4 
        ).finished();
        pose_prior_noise = pose_prior_noise.array().sqrt().matrix(); // Calc sigmas from variances
        vector<Timestep3D> timesteps = convert_into_timesteps(odomFactors3d, measFactors3d);
        slam::SLAM3D slam_sys{};
        // double ic_prob, int optimization_rate, const gtsam::Vector &pose_prior_noise, const gtsam::Vector &lmk_prior_noise
        slam_sys.initialize(ic_prob, optimization_rate, pose_prior_noise);
        int tot_timesteps = timesteps.size();
        for (const auto& timestep : timesteps) {
            cout << "Processing timestep " << timestep.step << ", " << double(timestep.step)/tot_timesteps*100.0 << "\% complete\n";
            slam_sys.processOdomMeasurementScan(
                timestep.odom, timestep.measurements
            );
        }
    } else {
        double ic_prob = 0.9707091134651118; // chi2.cdf(3**2, 3)
        gtsam::Vector pose_prior_noise = Vector3(1e-6, 1e-6, 1e-8);
        pose_prior_noise = pose_prior_noise.array().sqrt().matrix(); // Calc sigmas from variances
        cout << "Start converting into timesteps!\n";
        vector<Timestep2D> timesteps = convert_into_timesteps(odomFactors2d, measFactors2d);
        cout << "Done converting into timesteps!\n";
        slam::SLAM2D slam_sys{};
        // double ic_prob, int optimization_rate, const gtsam::Vector &pose_prior_noise, const gtsam::Vector &lmk_prior_noise
        slam_sys.initialize(ic_prob, optimization_rate, pose_prior_noise);
        cout << "SLAM system initialized!\n";
        int tot_timesteps = timesteps.size();
        for (const auto& timestep : timesteps) {
            cout << "Processing timestep " << timestep.step << ", " << double(timestep.step)/tot_timesteps*100.0 << "\% complete\n";
            slam_sys.processOdomMeasurementScan(
                timestep.odom, timestep.measurements
            );
        }
    }
}