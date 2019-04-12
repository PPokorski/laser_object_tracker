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

#ifndef LASER_OBJECT_TRACKER_UTILS_HPP
#define LASER_OBJECT_TRACKER_UTILS_HPP

#include "laser_object_tracker/data_types/definitions.hpp"

namespace test {
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

#endif  // LASER_OBJECT_TRACKER_UTILS_HPP
