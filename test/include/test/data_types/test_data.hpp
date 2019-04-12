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

#ifndef LASER_OBJECT_TRACKER_TEST_DATA_HPP
#define LASER_OBJECT_TRACKER_TEST_DATA_HPP

#include <cmath>

#include "laser_object_tracker/data_types/definitions.hpp"

namespace test {
struct ReferenceFragment {
    laser_object_tracker::data_types::LaserScanType laser_scan_;
    laser_object_tracker::data_types::OcclusionType occlusion_array_;
    laser_object_tracker::data_types::PointCloudType laser_scan_cloud_;
};

inline ReferenceFragment getFragment1() {
    ReferenceFragment fragment;

    fragment.laser_scan_.header.frame_id = "test";
    fragment.laser_scan_.range_max = 10.0;
    fragment.laser_scan_.ranges.push_back(1.0f);

    fragment.occlusion_array_.push_back(false);

    fragment.laser_scan_cloud_.points.push_back({1.0, 0.0, 0.0});

    return fragment;
}

inline ReferenceFragment getFragment11() {
    ReferenceFragment fragment;

    fragment.laser_scan_.header.frame_id = "test";
    fragment.laser_scan_.angle_min = M_PI_4;
    fragment.laser_scan_.angle_max = M_PI_4;
    fragment.laser_scan_.range_max = 10.0;
    fragment.laser_scan_.ranges.push_back(1.0f);

    fragment.occlusion_array_.push_back(false);

    fragment.laser_scan_cloud_.points.push_back({0.707107, 0.707107, 0.0});

    return fragment;
}

inline ReferenceFragment getFragment2() {
    ReferenceFragment fragment;
    fragment.laser_scan_.header.frame_id = "test";
    fragment.laser_scan_.angle_min = -M_PI_2;
    fragment.laser_scan_.angle_max = M_PI_2;
    fragment.laser_scan_.angle_increment = M_PI / 10;
    fragment.laser_scan_.range_max = 10.0;
    fragment.laser_scan_.ranges = {1.0, 2.0, 3.0, 2.0, 5.0, 6.0, 1.0, 2.0, 7.0, 5.0, 4.3};

    fragment.occlusion_array_.resize(11, false);

    fragment.laser_scan_cloud_.points = {
            {0.0, -1.0, 0.0},
            {0.618034, -1.90211, 0.0},
            {1.76336, -2.42705, 0.0},
            {1.61803, -1.17557, 0.0},
            {4.75528, -1.54509, 0.0},
            {6.0, 0.0, 0.0},
            {0.951057, 0.309017, 0.0},
            {1.61803, 1.17557, 0.0},
            {4.1145, 5.66312, 0.0},
            {1.54509, 4.75528, 0.0},
            {0.0, 4.3, 0.0}
    };

    return fragment;
}

inline ReferenceFragment getFragmentUnique1() {
    ReferenceFragment fragment;
    fragment.laser_scan_.header.frame_id = "test";
    fragment.laser_scan_.angle_min = 0;
    fragment.laser_scan_.angle_max = M_PI;
    fragment.laser_scan_.angle_increment = M_PI;
    fragment.laser_scan_.range_max = 10.0;
    fragment.laser_scan_.ranges = {1.0, 2.0};

    fragment.occlusion_array_.resize(2, false);

    fragment.laser_scan_cloud_.points = {
            {1.0, 0.0, 0.0},
            {-2.0, 0.0, 0.0}
    };

    return fragment;
}

inline ReferenceFragment getFragmentUnique2() {
    ReferenceFragment fragment;
    fragment.laser_scan_.header.frame_id = "test";
    fragment.laser_scan_.angle_min = 0;
    fragment.laser_scan_.angle_max = M_PI;
    fragment.laser_scan_.angle_increment = M_PI / 9.0;
    fragment.laser_scan_.range_max = 20.0;
    fragment.laser_scan_.ranges = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};

    fragment.occlusion_array_.resize(10, false);

    fragment.laser_scan_cloud_.points = {
            {1.0, 0.0, 0.0},
            {1.8793852416, 0.6840402867, 0.0},
            {2.2981333294, 1.9283628291, 0.0},
            {2.0, 3.4641016151, 0.0},
            {0.8682408883, 4.9240387651, 0.0},
            {-1.041889066, 5.9088465181, 0.0},
            {-3.5, 6.0621778265, 0.0},
            {-6.128355545, 5.1423008775, 0.0},
            {-8.4572335871, 3.0781812899, 0.0},
            {-10.0, 1.22464679914735E-15, 0.0}
    };

    return fragment;
}

}  // namespace test

#endif  // LASER_OBJECT_TRACKER_TEST_DATA_HPP
