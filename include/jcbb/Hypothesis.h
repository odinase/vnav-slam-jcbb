#ifndef HYPOTHESIS_H
#define HYPOTHESIS_H

#include <vector>
#include <optional>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <memory>
#include <unordered_set>
#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
// #include "MarginalMocks.h"

namespace jcbb
{

    // Config here between mock-up or GTSAM version.
    using Marginals = gtsam::Marginals;
    using JointMarginal = gtsam::JointMarginal;

    struct Association
    {
        explicit Association(int m);
        Association(int m, gtsam::Key l, const gtsam::Matrix& Hx, const gtsam::Matrix& Hl, const gtsam::Vector& error);
        Association(int m, gtsam::Key l); // For ML
        typedef std::shared_ptr<Association> shared_ptr;
        int measurement;
        std::optional<gtsam::Key> landmark;
        gtsam::Matrix Hx;
        gtsam::Matrix Hl;
        bool associated() const { return bool(landmark); }
        // double nis(const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S);
        gtsam::Vector error;
    };


    class Hypothesis
    {
        private:
        double nis_;
        gtsam::FastVector<Association::shared_ptr> assos_;

        public:
        Hypothesis(const gtsam::FastVector<Association::shared_ptr> &associations, double nis) : assos_(associations), nis_(nis) {}
        int num_associations() const;
        int num_measurements() const;
        void set_nis(double nis) {nis_ = nis;}
        double get_nis() const {return nis_;}

    gtsam::KeyVector associated_landmarks() const;
        // Needed for min heap
        bool operator<(const Hypothesis &rhs) const;

        // Needed for min heap
        bool operator>(const Hypothesis &rhs) const;

        // Added for completion of comparision operators
        bool operator==(const Hypothesis &rhs) const;

        // Added for completion of comparision operators
        bool operator<=(const Hypothesis &rhs) const;

        // Added for completion of comparision operators
        bool operator>=(const Hypothesis &rhs) const;

        bool better_than(const Hypothesis &other) const;

        static Hypothesis empty_hypothesis();

        void extend(const Association::shared_ptr &a);

        Hypothesis extended(const Association::shared_ptr &a) const;

        const gtsam::FastVector<Association::shared_ptr>& associations() const {
            return assos_;
        } 

    gtsam::FastVector<std::pair<int, gtsam::Key>> measurement_landmark_associations() const;
    void fill_with_unassociated_measurements(int tot_num_measurements);
    };
    
} // namespace jcbb

#endif // HYPOTHESIS_H