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

#ifndef LASER_OBJECT_TRACKER_TEST_SEGMENTATION_TEST_DATA_BD_HPP
#define LASER_OBJECT_TRACKER_TEST_SEGMENTATION_TEST_DATA_BD_HPP

#include "test/segmentation/test_data.hpp"

namespace test {

inline ReferenceSegmentation getSegmentationBD2() {
  ReferenceSegmentation segmentation;
  laser_object_tracker::data_types::LaserScanFragment::LaserScanFragmentFactory factory;

  auto laser_scan = generateLaserScan({1.0, 1.1, 1.2, 1.7, 2.5, 2.8, 2.9, 4.0, 5.0, 5.1});

  segmentation.fragment_ = factory.fromLaserScan(laser_scan);
  segmentation.threshold_ = 0.5;

  auto segment = generateLaserScan(laser_scan, 0, 3);
  segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

  segment = generateLaserScan(laser_scan, 4, 6);
  segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

  segment = generateLaserScan(laser_scan, 7, 7);
  segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

  segment = generateLaserScan(laser_scan, 8, 9);
  segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

  return segmentation;
}

inline ReferenceSegmentation getSegmentationBD3() {
  ReferenceSegmentation segmentation;
  laser_object_tracker::data_types::LaserScanFragment::LaserScanFragmentFactory factory;

  auto laser_scan = generateLaserScan({1.0, 1.1, 1.2, 1.7, 2.5, 2.8, 2.9, 4.0, 5.0, 5.1});

  segmentation.fragment_ = factory.fromLaserScan(laser_scan);
  segmentation.threshold_ = 1.0;

  auto segment = generateLaserScan(laser_scan, 0, 6);
  segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

  segment = generateLaserScan(laser_scan, 7, 9);
  segmentation.segmented_fragment_.push_back(factory.fromLaserScan(segment));

  return segmentation;
}
}  // namespace test

#endif  // LASER_OBJECT_TRACKER_TEST_SEGMENTATION_TEST_DATA_BD_HPP
