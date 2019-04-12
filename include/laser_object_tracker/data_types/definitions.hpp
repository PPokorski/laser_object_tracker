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

#ifndef LASER_OBJECT_TRACKER_DEFINITIONS_HPP
#define LASER_OBJECT_TRACKER_DEFINITIONS_HPP

// STD
#include <vector>

// ROS
#include <sensor_msgs/LaserScan.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace laser_object_tracker {
namespace data_types {

/**
* @brief Proxy class for bool built-in type, used becuse std::vector<bool> is specialized template container,
* not fully conforming to the standard.
*/
class Bool {
 public:
    /**
     * @brief Implicit conversion c-tor from bool type.
     * @param value Value to be initialized to.
     */
    Bool(bool value) : value_(value) {}

    /**
     * @brief Conversion operator to type bool&.
     * @return Reference to underlying bool value.
     */
    operator bool&() {
        return value_;
    }
    /**
     * @brief Conversion operator to type const bool&.
     * @return Const reference to underlying bool value.
     */
    operator const bool&() const {
        return value_;
    }

 private:
    bool value_;
};

using LaserScanType = sensor_msgs::LaserScan;
using OcclusionType = std::vector<Bool>;
using PointCloudType = pcl::PointCloud<pcl::PointXYZ>;

/**
 * @brief Proxy class or aggregator for a single element of LaserScanFragment container, e.g. single measurement.
 * Each element consists of polar (angle and range members) and cartesian (point member) coordinates
 * corresponding to the current measure. Also provides information whether a measurement is occluded or no.
 */
class FragmentElement {
 public:
    FragmentElement(double angle,
            LaserScanType::_ranges_type::reference range,
            OcclusionType::reference is_occluded,
            PointCloudType::PointType& point,
            bool less_than_min,
            bool more_than_max) :
            angle_(angle),
            range_(range),
            is_occluded_(is_occluded),
            point_(point),
            less_than_min_(less_than_min),
            more_than_max_(more_than_max) {}

    double getAngle() const {
        return angle_;
    }

    LaserScanType::_ranges_type::reference range() {
        return range_;
    }
    LaserScanType::_ranges_type::const_reference range() const {
        return range_;
    }

    OcclusionType::reference isOccluded() {
        return is_occluded_;
    }
    OcclusionType::const_reference isOccluded() const {
        return is_occluded_;
    }

    PointCloudType::PointType& point() {
        return point_;
    }
    const PointCloudType::PointType& point() const {
        return point_;
    }

    bool isValid() const {
        return !less_than_min_ && !more_than_max_;
    }

    bool lessThanMin() const {
        return less_than_min_;
    }

    bool moreThanMax() const {
        return more_than_max_;
    }

 private:
    double angle_;
    LaserScanType::_ranges_type::reference range_;
    OcclusionType::reference is_occluded_;
    PointCloudType::PointType& point_;

    bool less_than_min_;
    bool more_than_max_;
};

}  // namespace data_types
}  // namespace laser_object_tracker

#endif  // LASER_OBJECT_TRACKER_DEFINITIONS_HPP
