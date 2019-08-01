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

#include "laser_object_tracker/feature_extraction/features/segment_2d.hpp"

#include "test/utils.hpp"

using namespace laser_object_tracker::feature_extraction::features;

TEST(Segment2DTest, AccessorsTest) {
  Segment2D segment;

  Point2D expected(1.0, 1.0);
  segment.setStart(expected);
  EXPECT_TRUE(expected.isApprox(segment.getStart(), test::PRECISION<double>))
      << "Expected vector is: " << expected.transpose()
      << " but actual is: " << segment.getStart();

  expected << 3.0, 3.0;
  segment.setEnd(expected);
  EXPECT_TRUE(expected.isApprox(segment.getEnd(), test::PRECISION<double>))
            << "Expected vector is: " << expected.transpose()
            << " but actual is: " << segment.getEnd();

  EXPECT_NEAR(M_PI_4, segment.getOrientation(), test::PRECISION<double>);

  segment.setIsStartOccluded(true);
  EXPECT_TRUE(segment.isStartOccluded());

  segment.setIsStartOccluded(false);
  EXPECT_FALSE(segment.isStartOccluded());

  segment.setIsEndOccluded(true);
  EXPECT_TRUE(segment.isEndOccluded());

  segment.setIsEndOccluded(false);
  EXPECT_FALSE(segment.isEndOccluded());
}

TEST(Segment2DTest, ConstuctorTest) {
  Point2D start_expected(-1.0, 3.0),
          end_expected(32.0, -21.3);

  Segment2D segment(start_expected, end_expected, true, false);

  EXPECT_TRUE(start_expected.isApprox(segment.getStart(), test::PRECISION<double>))
            << "Expected vector is: " << start_expected.transpose()
            << " but actual is: " << segment.getStart();

  EXPECT_TRUE(end_expected.isApprox(segment.getEnd(), test::PRECISION<double>))
            << "Expected vector is: " << end_expected.transpose()
            << " but actual is: " << segment.getEnd();

  EXPECT_TRUE(segment.isStartOccluded());
  EXPECT_FALSE(segment.isEndOccluded());
}

TEST(Segment2DTest, LineConversionTest) {
  Point2D start(3.0, -3.0),
          end(43.0, -32.1);

  Segment2D segment(start, end, true, true);

  Line2D expected_line = Line2D::Through(start, end);
  EXPECT_TRUE(expected_line.isApprox(segment.line(), test::PRECISION<double>))
            << "Expected line is: " << expected_line.coeffs().transpose()
            << " but actual is: " << segment.line().coeffs().transpose();

  start << 0.0, 12.0;
  end << 2143.1, -301.0;
  segment.setStart(start);
  segment.setEnd(end);
  expected_line = Line2D::Through(start, end);
  EXPECT_TRUE(expected_line.isApprox(segment.line(), test::PRECISION<double>))
            << "Expected line is: " << expected_line.coeffs().transpose()
            << " but actual is: " << segment.line().coeffs().transpose();
}

TEST(Segment2DTest, LengthTest) {
  Segment2D segment({0.0, 0.0}, {4.0, 4.0}, true, true);
  EXPECT_NEAR(4 * M_SQRT2, segment.length(), test::PRECISION<double>);

  segment.setStart({32.0, 12.4});
  segment.setEnd({53.0, -31.0});
  EXPECT_NEAR(48.2137, segment.length(), test::PRECISION<double>);
}

TEST(Segment2DTest, OrientationTest) {
  Segment2D segment({0.0, 0.0}, {1.0, 1.0}, true, true);
  EXPECT_NEAR(M_PI_4, segment.getOrientation(), test::PRECISION<double>);

  segment.setEnd({-1.0, 1.0});
  EXPECT_NEAR(3 * M_PI_4, segment.getOrientation(), test::PRECISION<double>);

  segment.setEnd({-1.0, -1.0});
  EXPECT_NEAR(-3 * M_PI_4, segment.getOrientation(), test::PRECISION<double>);
}
