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

#include "laser_object_tracker/tracking/object_matching/fast_object_matching.hpp"

#include "test/utils.hpp"

using namespace laser_object_tracker::feature_extraction::features;
using namespace laser_object_tracker::tracking::object_matching;

TEST(FastObjectMatchingTest, BufferingTest) {
  FastObjectMatching object_matching(2.0, 1.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(0, object_matching.size());

  EXPECT_FALSE(object_matching.isReady(ros::Time(0.0)));
  EXPECT_FALSE(object_matching.isReady(ros::Time(10.0)));

  Object object;
  object.setTimestamp(ros::Time(1.0));
  object_matching.buffer({object});
  EXPECT_EQ(1, object_matching.size());

  EXPECT_FALSE(object_matching.isReady(ros::Time(1.9)));
  EXPECT_TRUE(object_matching.isReady(ros::Time(2.1)));

  object_matching.popOutdated(ros::Time(ros::Time(2.9)));
  EXPECT_EQ(1, object_matching.size());

  object_matching.popOutdated(ros::Time(ros::Time(3.1)));
  EXPECT_EQ(0, object_matching.size());

  object_matching.buffer({object});
  object.setTimestamp(ros::Time(2.0));
  object_matching.buffer({object});
  object.setTimestamp(ros::Time(3.0));
  object_matching.buffer({object});

  EXPECT_EQ(3, object_matching.size());
  EXPECT_FALSE(object_matching.isReady(ros::Time(1.9)));
  EXPECT_TRUE(object_matching.isReady(ros::Time(2.1)));

  object_matching.popOutdated(ros::Time(3.5));
  EXPECT_EQ(2, object_matching.size());

  object_matching.popOutdated(ros::Time(6.0));
  EXPECT_EQ(0, object_matching.size());
}

TEST(FastObjectMatchingTest, SingleSegmentMatchingTest) {
  FastObjectMatching object_matching(0.0, 0.0, 0.2, M_PI_4, 0.0);
  Object object;
  object.setSegments({
    {{0.0, 0.0}, {1.0, 1.0}}
  });
  object_matching.buffer({object});

  // Same segment
  EXPECT_TRUE(object_matching.isMatched(object));

  // Distance threshold
  object.setSegments({
    {{0.2, 0.2}, {1.0, 1.0}}
  });
  EXPECT_TRUE(object_matching.isMatched(object));

  object.setSegments({
    {{0.0, 0.0}, {1.2, 1.2}}
  });
  EXPECT_TRUE(object_matching.isMatched(object));

  object.setSegments({
    {{0.2, 0.2}, {1.2, 1.2}}
  });
  EXPECT_FALSE(object_matching.isMatched(object));

  object.setSegments({
    {{0.0, 0.0}, {1.0, -0.1}}
  });
  EXPECT_FALSE(object_matching.isMatched(object));

  object.setSegments({
    {{0.0, 1.1}, {1.0, 1.0}}
  });
  EXPECT_FALSE(object_matching.isMatched(object));

  object.setSegments({
    {{1.1, 0.0}, {1.0, 1.0}}
  });
  EXPECT_FALSE(object_matching.isMatched(object));
}

TEST(FastObjectMatchingTest, MultipleSegmentMatchingTest) {
  FastObjectMatching object_matching(0.0, 0.0, 0.2, M_PI_4, 0.0);
  Object object;
  object.setSegments({
    {{10.0, 10.0}, {9.0, 9.0}},
    {{8.0, 8.0}, {9.0, 8.0}}
  });
  Object object_2;
  object_2.setSegments({
    {{-3.0, 0.0}, {3.0, 0.0}}
  });
  object_matching.buffer({object, object_2});

  object.setSegments({
    {{0.0, 0.0}, {1.0, 1.0}}
  });
  object_matching.buffer({object});

  object.setSegments({
    {{0.0, 0.0}, {1.0, 1.0}}
  });
  EXPECT_FALSE(object_matching.isMatched(object));

  object.setSegments({
    {{10.0, 10.0}, {9.0, 9.0}}
  });
  EXPECT_TRUE(object_matching.isMatched(object));

  object.setSegments({
    {{8.0, 8.0}, {9.0, 8.0}}
  });
  EXPECT_TRUE(object_matching.isMatched(object));

  object.setSegments({
    {{-3.0, 0.0}, {3.0, 0.0}}
  });
  EXPECT_TRUE(object_matching.isMatched(object));

  object.setSegments({
    {{8.2, 8.2}, {9.2, 8.2}}
  });
  EXPECT_FALSE(object_matching.isMatched(object));

  object.setSegments({
    {{8.0, 8.0}, {8.0, 9.0}}
  });
  EXPECT_FALSE(object_matching.isMatched(object));
}

TEST(FastObjectMatchingTest, SingleCornerMatchingTest) {
  FastObjectMatching object_matching(0.0, 0.0, 0.2, M_PI_4, 1.0);

  Object object;
  object.setCorners({
    {{1.0, 1.0}, 1.0, 2.0, 1.0, 1.0, 1.0}
  });

  object_matching.buffer({object});
  EXPECT_TRUE(object_matching.isMatched(object));

  object.setCorners({
    {{1.1, 1.1}, 1.5, 2.0, 2.0, 1.0, 1.0}
  });
  EXPECT_TRUE(object_matching.isMatched(object));

  object.setCorners({
    {{1.5, 1.5}, 1.0, 2.0, 1.0, 1.0, 1.0}
  });
  EXPECT_FALSE(object_matching.isMatched(object));

  object.setCorners({
    {{1.0, 1.0}, 2.0, 2.0, 1.0, 1.0, 1.0}
  });
  EXPECT_TRUE(object_matching.isMatched(object));

  object.setCorners({
    {{1.0, 1.0}, 1.0, 2.0, 2.5, 1.0, 1.0}
  });
  EXPECT_TRUE(object_matching.isMatched(object));

  object.setCorners({
    {{1.0, 1.0}, 2.0, 2.0, 2.5, 1.0, 1.0}
  });
  EXPECT_FALSE(object_matching.isMatched(object));
}
