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

#ifndef LASER_OBJECT_TRACKER_TEST_SEGMENTATION_TEST_DATA_ABD_HPP
#define LASER_OBJECT_TRACKER_TEST_SEGMENTATION_TEST_DATA_ABD_HPP

#include "test/segmentation/test_data.hpp"

namespace test {

inline ReferenceSegmentation getSegmentationABD2() {
    ReferenceSegmentation segmentation;
    laser_object_tracker::data_types::LaserScanFragment::LaserScanFragmentFactory factory;

    // 5 elements, resolution is PI/2
    auto laser_scan = generateLaserScan({1.0, 1.5, 3.1, 4.0, 9.0});

    segmentation.fragment_ = factory.fromLaserScan(laser_scan);
    // Incidence angle is PI, so distance threshold will be simply a current range
    segmentation.threshold_ = M_PI;
    segmentation.resolution_ = 0.0;

    auto segment = generateLaserScan(laser_scan, 0, 1);
    segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

    segment = generateLaserScan(laser_scan, 2, 3);
    segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

    segment = generateLaserScan(laser_scan, 4, 4);
    segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

    return segmentation;
}

inline ReferenceSegmentation getSegmentationABD3() {
    ReferenceSegmentation segmentation;
    laser_object_tracker::data_types::LaserScanFragment::LaserScanFragmentFactory factory;

    // 9 elements, resolution is PI/4
    auto laser_scan = generateLaserScan({1.0, 1.7, 2.9, 5.0, 8.5, 14.5, 24.8, 42.3, 72.3},
            -M_PI,
            M_PI,
            "",
            0.0,
            100.0);

    segmentation.fragment_ = factory.fromLaserScan(laser_scan);
    // Incidence angle is 3 PI/4, so distance threshold will be range * sqrt(2)/2
    segmentation.threshold_ = 3 * M_PI_4;
    segmentation.resolution_ = 0.0;

    auto segment = generateLaserScan(laser_scan, 0, 2);
    segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

    segment = generateLaserScan(laser_scan, 3, 5);
    segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

    segment = generateLaserScan(laser_scan, 6, 7);
    segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

    segment = generateLaserScan(laser_scan, 8, 8);
    segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

    return segmentation;
}

inline ReferenceSegmentation getSegmentationABD4() {
    ReferenceSegmentation segmentation;
    laser_object_tracker::data_types::LaserScanFragment::LaserScanFragmentFactory factory;

    // 5 elements, resolution is PI/2
    auto laser_scan = generateLaserScan({1.0, 1.5, 3.7, 8.1, 9.9});

    segmentation.fragment_ = factory.fromLaserScan(laser_scan);
    // Incidence angle is PI, so distance threshold will be simply a current range
    segmentation.threshold_ = M_PI;
    segmentation.resolution_ = 0.2;

    auto segment = generateLaserScan(laser_scan, 0, 1);
    segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

    segment = generateLaserScan(laser_scan, 2, 2);
    segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

    segment = generateLaserScan(laser_scan, 3, 4);
    segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

    return segmentation;
}

}  // namespace test

#endif  // LASER_OBJECT_TRACKER_TEST_SEGMENTATION_TEST_DATA_ABD_HPP
