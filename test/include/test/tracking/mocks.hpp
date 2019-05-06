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

#ifndef TEST_TEST_TRACKING_MOCKS_HPP
#define TEST_TEST_TRACKING_MOCKS_HPP

#include <gmock/gmock.h>

#include "laser_object_tracker/tracking/base_tracking.hpp"

namespace test {
class MockTracking : public laser_object_tracker::tracking::BaseTracking {
 public:
  MockTracking() : BaseTracking(0, 0) {}

  MockTracking(int s, int m) : BaseTracking(0, 0) {}

  MOCK_METHOD1(initFromState, void(const laser_object_tracker::feature_extraction::features::Feature&));

  MOCK_METHOD1(initFromMeasurement, void(const laser_object_tracker::feature_extraction::features::Feature&));

  MOCK_METHOD0(predict, void());

  MOCK_METHOD1(update, void(const laser_object_tracker::feature_extraction::features::Feature&));

  MOCK_CONST_METHOD0(getStateVector, Eigen::VectorXd());

  std::unique_ptr<BaseTracking> clone() const override {
    return std::unique_ptr<BaseTracking>(new MockTracking());
  }
};

class MockTrackerRejection : public laser_object_tracker::tracking::BaseTrackerRejection {
  MOCK_CONST_METHOD1(invalidate, bool(const laser_object_tracker::tracking::BaseTracking& tracker));

  std::unique_ptr<BaseTrackerRejection> clone() const override {
    return std::unique_ptr<BaseTrackerRejection>(new MockTrackerRejection());
  }
};

}  // namespace test

#endif //TEST_TEST_TRACKING_MOCKS_HPP
