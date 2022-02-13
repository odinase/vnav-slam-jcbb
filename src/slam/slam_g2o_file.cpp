
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/slam/PoseToPointFactor.h>
// #include "slam/utils.h"
#include "slam/slam_g2o_file.h"
#include "slam/utils_g2o.h"
#include <cmath>
#include <fstream>
#include <tuple>
#include <algorithm>

using gtsam::symbol_shorthand::L; // gtsam/slam/dataset.cpp
using namespace std;
using namespace gtsam;

// SLAM_G2O_file::SLAM_G2O_file(const std::string& filename) : g2o_filename_(filename) {
//     boost::tie()
// }

template <class POSE>
using Odometry = BetweenFactor<POSE>;

using Odometry3D = Odometry<Pose3>;

template <class POSE, class POINT>
using Measurement = PoseToPointFactor<POSE, POINT>;

using Measurement3D = Measurement<Pose3, Point3>;

template <class POSE, class POINT>
struct Timestep
{
    int step;
    POSE odom;
    FastVector<POINT> measurements;
};

using Timestep2D = Timestep<Pose2, Point2>;
using Timestep3D = Timestep<Pose3, Point3>;


template <class POSE, class POINT>
std::vector<Timestep<POSE, POINT>> convert_into_timesteps(
    vector<boost::shared_ptr<BetweenFactor<POSE>>> &odomFactors,
    vector<boost::shared_ptr<PoseToPointFactor<POSE, POINT>>> &measFactors
)
{
    // Sort factors based on robot pose key, so that we can simply check when in time they should appear
    std::sort(
        odomFactors.begin(),
        odomFactors.end(), 
        [](const auto& lhs, const auto& rhs) {
            return symbolIndex(lhs->key1()) < symbolIndex(rhs->key1()); 
        }
    );
    std::sort(
        measFactors.begin(),
        measFactors.end(), 
        [](const auto& lhs, const auto& rhs) {
            return symbolIndex(lhs->key1()) < symbolIndex(rhs->key1()); 
        }
    );

    size_t odoms = odomFactors.size();
    uint64_t num_timesteps = odoms + 1; // There will always be one more robot pose than odometry factors since they're all between
    vector<Timestep<POSE, POINT>> timesteps;
    size_t curr_measurement = 0;
    size_t tot_num_measurements = measFactors.size();
    timesteps.reserve(num_timesteps);
    for (uint64_t t = 0; t < num_timesteps; t++) {
        Timestep<POSE, POINT> timestep;
        timestep.step = t;
        // Initialize first odom as identity, as we haven't moved yet
        if (t > 0) {
            timestep.odom = odomFactors[t-1]->measured();
        } else {
            timestep.odom = POSE();
        }

        // Extract measurements from current pose
        while (curr_measurement < tot_num_measurements && symbolIndex(measFactors[curr_measurement]->key1()) == t) {
            timestep.measurements.push_back(measFactors[curr_measurement]->measured());
            curr_measurement++;
        }
        
        timesteps.push_back(timestep);
    }

    return timesteps;
}

int main(const int argc, const char *argv[])
{
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
    if (is3D)
    {
        vector<Timestep<Pose3, Point3>> timesteps = convert_into_timesteps(odomFactors3d, measFactors3d);
        for (const auto& timestep : timesteps) {
            cout << "\n****************************\n";
            cout << "in timestep " << timestep.step <<":\n";
            cout << "odom:\n" << timestep.odom << "\n";
            cout << "measurements:\n";
            for (const auto measurement : timestep.measurements) {
                cout << measurement << "\n\n";
            }
        }
    } else {
        vector<Timestep<Pose2, Point2>> timesteps = convert_into_timesteps(odomFactors2d, measFactors2d);
        for (const auto& timestep : timesteps) {
            cout << "\n****************************\n";
            cout << "in timestep " << timestep.step <<":\n";
            cout << "odom:\n" << timestep.odom << "\n";
            cout << "measurements:\n";
            for (const auto measurement : timestep.measurements) {
                cout << measurement << "\n\n";
            }
        }
    }
}