/*********************************************************************
*
* BSD 3-Clause License
*
*  Copyright (c) 2019, Piotr Pokorski
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef LASER_OBJECT_TRACKER_TEST_UTILS_HPP
#define LASER_OBJECT_TRACKER_TEST_UTILS_HPP

#include "laser_object_tracker/data_types/definitions.hpp"
#include "laser_object_tracker/data_types/laser_scan_fragment.hpp"

namespace test {

inline laser_object_tracker::data_types::LaserScanType generateLaserScan(
        const std::vector<float>& ranges,
        float min_angle = -M_PI,
        float max_angle = M_PI,
        const std::string& frame = "",
        float min_range = 0.0,
        float max_range = 10.0) {
    laser_object_tracker::data_types::LaserScanType laser_scan;
    laser_scan.header.frame_id = frame;
    laser_scan.angle_min = min_angle;
    laser_scan.angle_max = max_angle;
    laser_scan.angle_increment = ranges.size() == 1? 0.0f : (max_angle - min_angle) / (ranges.size() - 1);
    laser_scan.range_min = min_range;
    laser_scan.range_max = max_range;
    laser_scan.ranges = ranges;

    return laser_scan;
}

inline laser_object_tracker::data_types::LaserScanType generateLaserScan(
        const laser_object_tracker::data_types::LaserScanType& prototype,
        long first_element,
        long last_element) {
    laser_object_tracker::data_types::LaserScanType laser_scan;
    laser_scan.header = prototype.header;
    laser_scan.angle_increment = prototype.angle_increment;
    laser_scan.range_min = prototype.range_min;
    laser_scan.range_max = prototype.range_max;

    laser_scan.angle_min = prototype.angle_min + first_element * prototype.angle_increment;
    laser_scan.angle_max = prototype.angle_min + last_element * prototype.angle_increment;

    for (long i = first_element; i <= last_element; ++i)
    {
        laser_scan.ranges.push_back(prototype.ranges.at(i));
    }

    return laser_scan;
}

template<class T>
constexpr static T PRECISION = T(0.0001);

template<class T>
inline bool close(const T& lhs, const T& rhs) {
    return std::abs(lhs - rhs) <= PRECISION<T>;
}

inline bool compare(const std_msgs::Header& lhs,
             const std_msgs::Header& rhs) {
    return lhs.frame_id == rhs.frame_id &&
           lhs.stamp == rhs.stamp;
}

inline bool compare(const laser_object_tracker::data_types::LaserScanType& lhs,
                    const laser_object_tracker::data_types::LaserScanType& rhs) {
    bool equal = compare(lhs.header, rhs.header) &&
                 close(lhs.angle_min, rhs.angle_min) &&
                 close(lhs.angle_max, rhs.angle_max) &&
                 close(lhs.angle_increment, rhs.angle_increment) &&
                 close(lhs.time_increment, rhs.time_increment) &&
                 close(lhs.scan_time, rhs.scan_time) &&
                 close(lhs.range_min, rhs.range_min) &&
                 close(lhs.range_max, rhs.range_max);
    if (!equal)
    {
        return equal;
    }
    else
    {
        return std::equal(lhs.ranges.cbegin(), lhs.ranges.cend(),
                          rhs.ranges.cbegin(), rhs.ranges.cend(),
                          close<laser_object_tracker::data_types::LaserScanType::_ranges_type::value_type>);
    }
}

inline bool compare(const laser_object_tracker::data_types::PointCloudType::PointType& lhs,
                    const laser_object_tracker::data_types::PointCloudType::PointType& rhs) {
    return close(lhs.x, rhs.x) &&
           close(lhs.y, rhs.y) &&
           close(lhs.z, rhs.z);
}

inline bool compare(const laser_object_tracker::data_types::PointCloudType& lhs,
                    const laser_object_tracker::data_types::PointCloudType& rhs) {
    return std::equal(lhs.begin(), lhs.end(),
                      rhs.begin(), rhs.end(),
                      [](const auto& lhs, const auto& rhs) {
                          return compare(lhs, rhs);
                      });
}

}  // namespace test

namespace laser_object_tracker {
namespace data_types {

inline bool operator==(const laser_object_tracker::data_types::LaserScanFragment& lhs,
                       const laser_object_tracker::data_types::LaserScanFragment& rhs) {
    return test::compare(lhs.laserScan(), rhs.laserScan()) &&
           lhs.occlusionVector() == rhs.occlusionVector() &&
           test::compare(lhs.pointCloud(), rhs.pointCloud());
}

}  // namespace data_types
}  // namespace laser_object_tracker

#endif  // LASER_OBJECT_TRACKER_TEST_UTILS_HPP
