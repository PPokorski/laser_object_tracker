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

#include "laser_object_tracker/tracking/multi_tracking.hpp"

#include "test/utils.hpp"
#include "test/data_association/mocks.hpp"
#include "test/tracking/mocks.hpp"

TEST(MultiTrackerTest, UpdateAndInitializeTracksTest) {
  auto data_association = std::make_unique<test::MockDataAssociation>();
  auto tracking = std::make_unique<test::MockTracking>();
  auto tracker_rejection = std::make_unique<test::MockTrackerRejection>();
  laser_object_tracker::tracking::MultiTracking multi_tracker(
      [](const auto& lhs, const auto& rhs) {return 0.0;},
      std::move(data_association),
      std::move(tracking),
      std::move(tracker_rejection));

  static constexpr int NO_ASSIGNMENT = laser_object_tracker::data_association::BaseDataAssociation::NO_ASSIGNMENT;

  std::vector<laser_object_tracker::feature_extraction::features::Feature> measurements;
  Eigen::VectorXi assignment_vector;

  measurements.resize(5);
  assignment_vector.setConstant(5, NO_ASSIGNMENT);
  multi_tracker.updateAndInitializeTracks(measurements, assignment_vector);
  ASSERT_EQ(5, multi_tracker.size());
  // All objects should be unique (with respect to pointer)
  for (auto it_1 = multi_tracker.begin(); it_1 != multi_tracker.end(); ++it_1) {
    for (auto it_2 = std::next(it_1); it_2 != multi_tracker.end(); ++it_2) {
      EXPECT_NE(*it_1, *it_2);
    }
  }

  measurements.resize(3);
  assignment_vector.setConstant(3, NO_ASSIGNMENT);
  multi_tracker.updateAndInitializeTracks(measurements, assignment_vector);
  ASSERT_EQ(8, multi_tracker.size());
  // All objects should be unique (with respect to pointer)
  for (auto it_1 = multi_tracker.begin(); it_1 != multi_tracker.end(); ++it_1) {
    for (auto it_2 = std::next(it_1); it_2 != multi_tracker.end(); ++it_2) {
      EXPECT_NE(*it_1, *it_2);
    }
  }

  measurements.resize(8);
  assignment_vector.resize(8);
  assignment_vector << 0, 1, 2, 3, 4, 5, 6, 7;
  for (const auto& tracker : multi_tracker) {
    auto& cast_tracker = dynamic_cast<test::MockTracking&>(*tracker);
    EXPECT_CALL(cast_tracker, update(testing::_));
  }

  multi_tracker.updateAndInitializeTracks(measurements, assignment_vector);
}
