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

#include <gtest/gtest.h>

#include "laser_object_tracker/segmentation/adaptive_breakpoint_detection.hpp"

#include "test/utils.hpp"
#include "test/segmentation/test_data_abd.hpp"

class AdaptiveBreakpointDetectionTestWithParam : public testing::TestWithParam<test::ReferenceSegmentation> {
 protected:
    std::shared_ptr<laser_object_tracker::segmentation::BaseSegmentation> segmentation_ptr_;
};

TEST(AdaptiveBreakpointDetectionTest, AccessorsTest) {
    laser_object_tracker::segmentation::AdaptiveBreakpointDetection abd(2.0, 3.0);
    EXPECT_NEAR(2.0, abd.getIncidenceAngle(), test::PRECISION<double>);
    EXPECT_NEAR(3.0, abd.getDistanceResolution(), test::PRECISION<double>);

    abd.setIncidenceAngle(10.0);
    EXPECT_NEAR(10.0, abd.getIncidenceAngle(), test::PRECISION<double>);
    EXPECT_NEAR(3.0, abd.getDistanceResolution(), test::PRECISION<double>);

    abd.setDistanceResolution(20.0);
    EXPECT_NEAR(10.0, abd.getIncidenceAngle(), test::PRECISION<double>);
    EXPECT_NEAR(20.0, abd.getDistanceResolution(), test::PRECISION<double>);
}

TEST_P(AdaptiveBreakpointDetectionTestWithParam, SegmentationTest) {
    test::ReferenceSegmentation reference = GetParam();

    segmentation_ptr_.reset(new laser_object_tracker::segmentation::AdaptiveBreakpointDetection(
            reference.threshold_, reference.resolution_));

    auto value = segmentation_ptr_->segment(reference.fragment_);
    EXPECT_EQ(reference.segmented_fragment_, value);
}

INSTANTIATE_TEST_CASE_P(AdaptiveBreakpointDetectionTestData,
        AdaptiveBreakpointDetectionTestWithParam,
        testing::Values(test::getSegmentationEmpty(),
                test::getSegmentation1(),
                test::getSegmentationABD2(),
                test::getSegmentationABD3(),
                test::getSegmentationABD4()));