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

#include "laser_object_tracker/filtering/obb_area_filter.hpp"

#include "test/utils.hpp"

TEST(OBBAreaFilterTest, ConstructorAndAccessorsTest) {
  laser_object_tracker::filtering::OBBAreaFilter filter(0.0, 10.0, 3.0);
  EXPECT_NEAR(0.0, filter.getMinArea(), test::PRECISION<double>);
  EXPECT_NEAR(10.0, filter.getMaxArea(), test::PRECISION<double>);
  EXPECT_NEAR(3.0, filter.getMinBoxDimension(), test::PRECISION<double>);

  filter.setMinArea(2.0);
  EXPECT_NEAR(2.0, filter.getMinArea(), test::PRECISION<double>);

  filter.setMaxArea(12.0);
  EXPECT_NEAR(12.0, filter.getMaxArea(), test::PRECISION<double>);

  filter.setMinBoxDimension(5.0);
  EXPECT_NEAR(5.0, filter.getMinBoxDimension(), test::PRECISION<double>);
}

TEST(OBBAreaFilterTest, ShouldFilterTest) {
  laser_object_tracker::filtering::OBBAreaFilter filter(std::numeric_limits<double>::epsilon(), 3.0, 0.0);
  laser_object_tracker::data_types::LaserScanFragment::LaserScanFragmentFactory factory;

  laser_object_tracker::data_types::LaserScanFragment fragment;
  EXPECT_TRUE(filter.shouldFilter(fragment));

  fragment = factory.fromLaserScan(test::generateLaserScan({1.0}));
  EXPECT_TRUE(filter.shouldFilter(fragment));

  fragment = factory.fromLaserScan(test::generateLaserScan({1.0, 1.0}));
  EXPECT_TRUE(filter.shouldFilter(fragment));

  filter.setMinArea(2.0);
  fragment = factory.fromLaserScan(test::generateLaserScan({1.0, 0.9, 1.0}, -M_PI_2, M_PI_2));
  EXPECT_TRUE(filter.shouldFilter(fragment));

  fragment = factory.fromLaserScan(test::generateLaserScan({1.0, 1.1, 1.0}, -M_PI_2, M_PI_2));
  EXPECT_FALSE(filter.shouldFilter(fragment));

  fragment = factory.fromLaserScan(test::generateLaserScan({1.0, 1.25, 1.0}, -M_PI_2, M_PI_2));
  EXPECT_FALSE(filter.shouldFilter(fragment));

  fragment = factory.fromLaserScan(test::generateLaserScan({1.0, 1.45, 1.0}, -M_PI_2, M_PI_2));
  EXPECT_FALSE(filter.shouldFilter(fragment));

  fragment = factory.fromLaserScan(test::generateLaserScan({1.0, 1.6, 1.0}, -M_PI_2, M_PI_2));
  EXPECT_TRUE(filter.shouldFilter(fragment));

  filter.setMinBoxDimension(1.1);
  filter.setMinArea(1.0);
  fragment = factory.fromLaserScan(test::generateLaserScan({1.0}));
  EXPECT_FALSE(filter.shouldFilter(fragment));
}
