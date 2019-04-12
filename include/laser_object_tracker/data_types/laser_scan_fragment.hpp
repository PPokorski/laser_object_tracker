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

#ifndef LASER_OBJECT_TRACKER_LASER_SCAN_FRAGMENT_HPP
#define LASER_OBJECT_TRACKER_LASER_SCAN_FRAGMENT_HPP

// ROS
#include <laser_geometry/laser_geometry.h>

// PROJECT
#include "laser_object_tracker/data_types/definitions.hpp"

namespace laser_object_tracker {
namespace data_types {

/**
 * @brief Container-type class consisting of LaserScan measurement, corresponding PointCloud and Occlusion vector
 * informing whether each point is occluded or not by its neighbours.
 */
class LaserScanFragment {
 public:
    using ContainerType = std::vector<FragmentElement>;
    using Iterator = ContainerType::iterator;
    using ConstIterator = ContainerType::const_iterator;

    /**
     * @brief Factory class for producing LaserScanFragments from LaserScanType
     */
    class LaserScanFragmentFactory {
     public:
        /**
         * @brief Copy factory method, initializes all internal data.
         * @param laser_scan LaserScan measurement. This value is being copied.
         * @return Fully initialized LaserScanFragment object
         */
        LaserScanFragment fromLaserScan(const LaserScanType& laser_scan);
        /**
         * @brief Move factory method, initializes all internal data.
         * @param laser_scan LaserScan measurement. This value is being moved from, e.g. invalidated.
         * @return Fully initialized LaserScanFragment object
         */
        LaserScanFragment fromLaserScan(LaserScanType&& laser_scan);

     private:
        /**
         * @brief Given fragment with initialized laser_scan, initialize rest of the fields
         * @param fragment Fragment to be initialized
         */
        void completeInitialization(LaserScanFragment& fragment);

        laser_geometry::LaserProjection laser_projector_;
    };

    /**
     * @brief Default c-tor. Creates empty fields
     */
    LaserScanFragment() = default;

    LaserScanFragment(const LaserScanFragment& other) noexcept;

    LaserScanFragment(LaserScanFragment&& other) noexcept;

    LaserScanFragment& operator=(const LaserScanFragment& other) noexcept;

    LaserScanFragment& operator=(LaserScanFragment&& other) noexcept;

    /**
     * @brief Constructs the container as a sub-container of other with range of [first, last)
     * @param other Prototype container
     * @param first First element of the range, included
     * @param last Last element of the range, not included
     */
    LaserScanFragment(const LaserScanFragment& other, long first, long last);

    /**
     *
     * @return Header of LaserScanType measurement
     */
    std_msgs::Header getHeader() const {
        return laser_scan_.header;
    }

    /**
     *
     * @return Min angle of the measurement (in polar coordinates)
     */
    double getAngleMin() const {
        return laser_scan_.angle_min;
    }

    /**
     *
     * @return Max angle of the measurement (in polar coordinates)
     */
    double getAngleMax() const {
        return laser_scan_.angle_max;
    }

    /**
     *
     * @return Angle resolution of the laser scanner
     */
    double getAngleIncrement() const {
        return laser_scan_.angle_increment;
    }

    /**
     *
     * @return Min range of the laser scanner
     */
    double getRangeMin() const {
        return laser_scan_.range_min;
    }

    /**
     *
     * @return Max range of the laser scanner
     */
    double getRangeMax() const {
        return laser_scan_.range_max;
    }

    /**
     * @brief Accessor to the underlying LaserScanType data.
     * @return Underlying LaserScanType data.
     */
    const LaserScanType& laserScan() const {
        return laser_scan_;
    }

    /**
     * @brief Accessor to the underlying OcclusionType data.
     * @return Underlying OcclusionType data.
     */
    const OcclusionType& occlusionVector() const {
        return occlusion_vector_;
    }

    /**
     * @brief Accessor to the underlying PointCloudType data.
     * @return Underlying PointCloudType data.
     */
    const PointCloudType& pointCloud() const {
        return laser_scan_cloud_;
    }

    /**
     *
     * @return Iterator pointing to the first measurement (counting from min to max angle)
     */
    Iterator begin();
    /**
     *
     * @return Const iterator pointing to the first measurement (counting from min to max angle)
     */
    ConstIterator cbegin() const;

    /**
     *
     * @return Iterator pointing to the one measurement beyond last (counting from min to max angle)
     */
    Iterator end();
    /**
     *
     * @return Const iterator pointing to the one measurement beyond last (counting from min to max angle)
     */
    ConstIterator cend() const;

    /**
     *
     * @return True if the container is empty, false otherwise
     */
    bool empty() const {
        return elements_.empty();
    }
    
    /**
     * 
     * @return The number of elements in the container 
     */
    long size() const {
        return elements_.size();
    }

 private:
    /**
     * @brief This method clear and initializes internal elements_ container
     */
    void initializeInternalContainer();

    LaserScanType laser_scan_;
    OcclusionType occlusion_vector_;
    PointCloudType laser_scan_cloud_;
    ContainerType elements_;
};
}  // namespace data_types
}  // namespace laser_object_tracker

#endif  // LASER_OBJECT_TRACKER_LASER_SCAN_FRAGMENT_HPP
