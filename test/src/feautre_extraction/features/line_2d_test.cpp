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

#include "laser_object_tracker/feature_extraction/features/line_2d.hpp"

#include "test/utils.hpp"

using namespace laser_object_tracker::feature_extraction::features;

TEST(Line2DTest, AngleBetweenLinesTest) {
  Line2D line_1 = Line2D::Through({0.0, 0.0}, {1.0, 0.0});
  Line2D line_2 = line_1;

  EXPECT_NEAR(0.0, angleBetweenLines(line_1, line_2), test::PRECISION<double>);

  line_2 = Line2D::Through({0.0, 0.0}, {1.0, 1.0});
  EXPECT_NEAR(M_PI_4, angleBetweenLines(line_1, line_2), test::PRECISION<double>);

  line_2 = Line2D::Through({0.0, 0.0}, {0.0, 1.0});
  EXPECT_NEAR(M_PI_2, angleBetweenLines(line_1, line_2), test::PRECISION<double>);

  line_2 = Line2D::Through({0.0, 0.0}, {-1.0, 1.0});
  EXPECT_NEAR(3 * M_PI_4, angleBetweenLines(line_1, line_2), test::PRECISION<double>);

  line_2 = Line2D::Through({0.0, 0.0}, {-1.0, 0.0});
  EXPECT_NEAR(M_PI, angleBetweenLines(line_1, line_2), test::PRECISION<double>);

  line_2 = Line2D::Through({0.0, 0.0}, {-1.0, -1.0});
  EXPECT_NEAR(3 * M_PI_4, angleBetweenLines(line_1, line_2), test::PRECISION<double>);

  line_2 = Line2D::Through({0.0, 0.0}, {0.0, -1.0});
  EXPECT_NEAR(M_PI_2, angleBetweenLines(line_1, line_2), test::PRECISION<double>);
}
