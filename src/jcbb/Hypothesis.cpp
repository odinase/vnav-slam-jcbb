#include <unordered_set>
#include <algorithm>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Key.h>

#include "jcbb/Hypothesis.h"

namespace jcbb
{
    Association::Association(int m) : measurement(m), landmark({}) {}
    Association::Association(int m, gtsam::Key l, const gtsam::Matrix &Hx, const gtsam::Matrix &Hl, const gtsam::Vector& error) : measurement(m), landmark(l), Hx(Hx), Hl(Hl), error(error) {}

    int Hypothesis::num_associations() const
    {
        // int n = 0;
        // for (const auto &a : assos_)
        // {
        //     if (a->associated())
        //     {
        //         n++;
        //     }
        // }
        // return n;
        return std::count_if(assos_.cbegin(), assos_.cend(), [](const Association::shared_ptr& a) {return a->associated();});
    }

    int Hypothesis::num_measurements() const
    {
        return assos_.size();
    }

    gtsam::KeyVector Hypothesis::associated_landmarks() const
    {
        gtsam::KeyVector landmarks;
        for (const auto &a : assos_)
        {
            if (a->associated())
            {
                landmarks.push_back(*a->landmark);
            }
        }
        return landmarks;
    }
    // Needed for min heap
    bool Hypothesis::operator<(const Hypothesis &rhs) const
    {
        return num_associations() > rhs.num_associations() || (num_associations() == rhs.num_associations() && nis_ < rhs.nis_);
    }

    // Needed for min heap
    bool Hypothesis::operator>(const Hypothesis &rhs) const
    {
        return num_associations() < rhs.num_associations() || (num_associations() == rhs.num_associations() && nis_ > rhs.nis_);
    }

    // Added for completion of comparision operators
    bool Hypothesis::operator==(const Hypothesis &rhs) const
    {
        return num_associations() == rhs.num_associations() && nis_ == rhs.nis_;
    }

    // Added for completion of comparision operators
    bool Hypothesis::operator<=(const Hypothesis &rhs) const
    {
        return *this < rhs || *this == rhs;
    }

    // Added for completion of comparision operators
    bool Hypothesis::operator>=(const Hypothesis &rhs) const
    {
        return *this > rhs || *this == rhs;
    }

    bool Hypothesis::better_than(const Hypothesis &other) const
    {
        return *this < other;
    }

    Hypothesis Hypothesis::empty_hypothesis()
    {
        return Hypothesis{{}, std::numeric_limits<double>::infinity()};
    }

    void Hypothesis::extend(const Association::shared_ptr &a)
    {
        assos_.push_back(a);
    }

    Hypothesis Hypothesis::extended(const Association::shared_ptr &a) const
    {
        Hypothesis h(*this);
        h.extend(a);
        return h;
    }

    gtsam::FastVector<std::pair<int, gtsam::Key>> Hypothesis::measurement_landmark_associations() const
    {
        gtsam::FastVector<std::pair<int, gtsam::Key>> measurement_landmark_associations;
        for (const auto &a : assos_)
        {
            if (a->associated())
            {
                measurement_landmark_associations.push_back({a->measurement,
                                                             *a->landmark});
            }
        }
    }

} // namespace jcbb
