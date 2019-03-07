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

#include "laser_object_tracker/data_types/laser_scan_fragment.hpp"

// ROS
#include <pcl_conversions/pcl_conversions.h>


namespace laser_object_tracker {
namespace data_types {

LaserScanFragment LaserScanFragment::LaserScanFragmentFactory::fromLaserScan(const LaserScanType& laser_scan) {
    LaserScanFragment fragment;
    fragment.laser_scan_ = laser_scan;

    sensor_msgs::PointCloud2 pcl2;
    laser_projector_.projectLaser(fragment.laser_scan_, pcl2);
    pcl::moveFromROSMsg(pcl2, fragment.laser_scan_cloud_);

    fragment.occlusion_vector_.resize(fragment.laser_scan_.ranges.size(), false);

    fragment.elements_.reserve(fragment.laser_scan_.ranges.size());
    for (int i = 0; i < fragment.laser_scan_.ranges.size(); ++i)
    {
        fragment.elements_.push_back({
            fragment.laser_scan_.angle_min + i * laser_scan.angle_increment,
            fragment.laser_scan_.ranges[i],
            fragment.occlusion_vector_[i],
            fragment.laser_scan_cloud_[i]
        });
    }

    return fragment;
}

LaserScanFragment LaserScanFragment::LaserScanFragmentFactory::fromLaserScan(LaserScanType&& laser_scan) {
    LaserScanFragment fragment;
    fragment.laser_scan_ = std::move(laser_scan);

    sensor_msgs::PointCloud2 pcl2;
    laser_projector_.projectLaser(fragment.laser_scan_, pcl2);
    pcl::moveFromROSMsg(pcl2, fragment.laser_scan_cloud_);

    fragment.occlusion_vector_.resize(fragment.laser_scan_.ranges.size(), false);

    fragment.elements_.reserve(fragment.laser_scan_.ranges.size());
    for (int i = 0; i < fragment.laser_scan_.ranges.size(); ++i)
    {
        fragment.elements_.push_back({
             fragment.laser_scan_.angle_min + i * laser_scan.angle_increment,
             fragment.laser_scan_.ranges[i],
             fragment.occlusion_vector_[i],
             fragment.laser_scan_cloud_[i]
         });
    }

    return fragment;
}

LaserScanFragment::Iterator LaserScanFragment::begin() {
    return elements_.begin();
}

LaserScanFragment::ConstIterator LaserScanFragment::cbegin() const {
    return elements_.cbegin();
}

LaserScanFragment::Iterator LaserScanFragment::end() {
    return elements_.end();
}

LaserScanFragment::ConstIterator LaserScanFragment::cend() const {
    return elements_.cend();
}
}  // namespace data_types
}  // namespace laser_object_tracker
