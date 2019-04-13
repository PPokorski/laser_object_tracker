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

#include "laser_object_tracker/data_types/laser_scan_fragment.hpp"

#include "test/utils.hpp"
#include "test/data_types/test_data.hpp"

class LaserScanFragmentTest : public testing::Test {
 protected:
  laser_object_tracker::data_types::LaserScanFragment::LaserScanFragmentFactory factory_;
};

class LaserScanFragmentTestWithParam : public LaserScanFragmentTest,
                                       public testing::WithParamInterface<test::ReferenceFragment> {
};

TEST_F(LaserScanFragmentTest, EmptyScan) {
  auto fragment = factory_.fromLaserScan({});

  EXPECT_TRUE(fragment.empty())
            << "Fragment is not empty where it should";
  EXPECT_EQ(0, fragment.size());

  EXPECT_TRUE(fragment.laserScan().ranges.empty())
            << "Laser scan is not empty where it should";
  EXPECT_TRUE(fragment.occlusionVector().empty())
            << "Occlusion vector is not empty where it should";
  EXPECT_TRUE(fragment.pointCloud().empty())
            << "Point cloud is not empty where it should";
}

TEST_F(LaserScanFragmentTest, SubrangeConstructorTest) {
  auto laser_scan = test::getFragmentUnique2().laser_scan_;
  auto fragment = factory_.fromLaserScan(laser_scan);

  laser_object_tracker::data_types::LaserScanFragment result(fragment, 0, 1);
  auto expected_result = factory_.fromLaserScan(test::generateLaserScan(laser_scan, 0, 0));
  EXPECT_EQ(expected_result, result);

  result = laser_object_tracker::data_types::LaserScanFragment(fragment, 1, 2);
  expected_result = factory_.fromLaserScan(test::generateLaserScan(laser_scan, 1, 1));
  EXPECT_EQ(expected_result, result);

  result = laser_object_tracker::data_types::LaserScanFragment(fragment, 1, 4);
  expected_result = factory_.fromLaserScan(test::generateLaserScan(laser_scan, 1, 3));
  EXPECT_EQ(expected_result, result);

  result = laser_object_tracker::data_types::LaserScanFragment(fragment, 3, 10);
  expected_result = factory_.fromLaserScan(test::generateLaserScan(laser_scan, 3, 9));
  EXPECT_EQ(expected_result, result);
}

TEST_F(LaserScanFragmentTest, SubrangeConstructorExceptionsTest) {
  auto fragment = factory_.fromLaserScan(test::getFragmentUnique2().laser_scan_);

  EXPECT_THROW(laser_object_tracker::data_types::LaserScanFragment(fragment, -1, 0), std::out_of_range);
  EXPECT_THROW(laser_object_tracker::data_types::LaserScanFragment(fragment, 0, 11), std::out_of_range);
  EXPECT_THROW(laser_object_tracker::data_types::LaserScanFragment(fragment, 0, 0), std::invalid_argument);
  EXPECT_THROW(laser_object_tracker::data_types::LaserScanFragment(fragment, 1, 1), std::invalid_argument);
  EXPECT_THROW(laser_object_tracker::data_types::LaserScanFragment(fragment, 4, 0), std::invalid_argument);
}

TEST_P(LaserScanFragmentTestWithParam, AccessorTest) {
  test::ReferenceFragment reference = GetParam();
  auto fragment = factory_.fromLaserScan(reference.laser_scan_);

  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               reference.laser_scan_.header, fragment.getHeader());
  EXPECT_NEAR(reference.laser_scan_.angle_min, fragment.getAngleMin(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.angle_max, fragment.getAngleMax(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.angle_increment, fragment.getAngleIncrement(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.range_min, fragment.getRangeMin(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.range_max, fragment.getRangeMax(), test::PRECISION<double>);
  EXPECT_FALSE(fragment.empty())
            << "Fragment should not be empty whern it's not";

  EXPECT_EQ(reference.laser_scan_.ranges.size(), fragment.size());
}

TEST_P(LaserScanFragmentTestWithParam, AccessorTestMove) {
  test::ReferenceFragment copy, reference;
  copy = reference = GetParam();
  auto fragment = factory_.fromLaserScan(std::move(copy.laser_scan_));

  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               reference.laser_scan_.header, fragment.getHeader());
  EXPECT_NEAR(reference.laser_scan_.angle_min, fragment.getAngleMin(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.angle_max, fragment.getAngleMax(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.angle_increment, fragment.getAngleIncrement(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.range_min, fragment.getRangeMin(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.range_max, fragment.getRangeMax(), test::PRECISION<double>);
  EXPECT_FALSE(fragment.empty())
            << "Fragment should not be empty whern it's not";

  EXPECT_EQ(reference.laser_scan_.ranges.size(), fragment.size());
}

TEST_P(LaserScanFragmentTestWithParam, DataTest) {
  test::ReferenceFragment reference = GetParam();
  auto fragment = factory_.fromLaserScan(reference.laser_scan_);

  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               fragment.laserScan(), reference.laser_scan_);
  EXPECT_EQ(fragment.occlusionVector(), reference.occlusion_array_);
  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               fragment.pointCloud(), reference.laser_scan_cloud_);

  EXPECT_EQ(fragment.laserScan().ranges.size(), fragment.occlusionVector().size())
            << "All data representations must have equal size";
  EXPECT_EQ(fragment.laserScan().ranges.size(), fragment.pointCloud().size())
            << "All data representations must have equal size";
}

TEST_P(LaserScanFragmentTestWithParam, DataTestMove) {
  test::ReferenceFragment reference = GetParam();
  test::ReferenceFragment copy = reference;

  auto fragment = factory_.fromLaserScan(std::move(copy.laser_scan_));

  EXPECT_TRUE(copy.laser_scan_.ranges.empty())
            << "After moving ranges should be empty";

  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               fragment.laserScan(), reference.laser_scan_);
  EXPECT_EQ(fragment.occlusionVector(), reference.occlusion_array_);
  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               fragment.pointCloud(), reference.laser_scan_cloud_);

  EXPECT_EQ(fragment.laserScan().ranges.size(), fragment.occlusionVector().size())
            << "All data representations must have equal size";
  EXPECT_EQ(fragment.laserScan().ranges.size(), fragment.pointCloud().size())
            << "All data representations must have equal size";
}

TEST_P(LaserScanFragmentTestWithParam, IteratorsTest) {
  test::ReferenceFragment reference = GetParam();
  auto fragment = factory_.fromLaserScan(reference.laser_scan_);

  // Begin Iterators
  EXPECT_NEAR(reference.laser_scan_.angle_min, fragment.begin()->getAngle(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.angle_min, fragment.cbegin()->getAngle(), test::PRECISION<double>);

  EXPECT_NEAR(reference.laser_scan_.ranges.front(), fragment.begin()->range(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.ranges.front(), fragment.cbegin()->range(), test::PRECISION<double>);

  EXPECT_EQ(reference.occlusion_array_.front(), fragment.begin()->isOccluded());
  EXPECT_EQ(reference.occlusion_array_.front(), fragment.cbegin()->isOccluded());

  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               reference.laser_scan_cloud_.front(), fragment.begin()->point());
  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               reference.laser_scan_cloud_.front(), fragment.cbegin()->point());

  // End Iterators
  EXPECT_NEAR(reference.laser_scan_.angle_max, (--fragment.end())->getAngle(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.angle_max, (--fragment.cend())->getAngle(), test::PRECISION<double>);

  EXPECT_NEAR(reference.laser_scan_.ranges.back(), (--fragment.end())->range(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.ranges.back(), (--fragment.cend())->range(), test::PRECISION<double>);

  EXPECT_EQ(reference.occlusion_array_.back(), (--fragment.end())->isOccluded());
  EXPECT_EQ(reference.occlusion_array_.back(), (--fragment.cend())->isOccluded());

  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               reference.laser_scan_cloud_.back(), (--fragment.end())->point());
  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               reference.laser_scan_cloud_.back(), (--fragment.cend())->point());
}

TEST_P(LaserScanFragmentTestWithParam, IteratorsTestMove) {
  test::ReferenceFragment reference = GetParam();
  test::ReferenceFragment copy = reference;

  auto fragment = factory_.fromLaserScan(std::move(copy.laser_scan_));

  // Begin Iterators
  EXPECT_NEAR(reference.laser_scan_.angle_min, fragment.begin()->getAngle(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.angle_min, fragment.cbegin()->getAngle(), test::PRECISION<double>);

  EXPECT_NEAR(reference.laser_scan_.ranges.front(), fragment.begin()->range(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.ranges.front(), fragment.cbegin()->range(), test::PRECISION<double>);

  EXPECT_EQ(reference.occlusion_array_.front(), fragment.begin()->isOccluded());
  EXPECT_EQ(reference.occlusion_array_.front(), fragment.cbegin()->isOccluded());

  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               reference.laser_scan_cloud_.front(), fragment.begin()->point());
  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               reference.laser_scan_cloud_.front(), fragment.cbegin()->point());

  // End Iterators
  EXPECT_NEAR(reference.laser_scan_.angle_max, (--fragment.end())->getAngle(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.angle_max, (--fragment.cend())->getAngle(), test::PRECISION<double>);

  EXPECT_NEAR(reference.laser_scan_.ranges.back(), (--fragment.end())->range(), test::PRECISION<double>);
  EXPECT_NEAR(reference.laser_scan_.ranges.back(), (--fragment.cend())->range(), test::PRECISION<double>);

  EXPECT_EQ(reference.occlusion_array_.back(), (--fragment.end())->isOccluded());
  EXPECT_EQ(reference.occlusion_array_.back(), (--fragment.cend())->isOccluded());

  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               reference.laser_scan_cloud_.back(), (--fragment.end())->point());
  EXPECT_PRED2([](const auto& lhs, const auto& rhs) { return test::compare(lhs, rhs); },
               reference.laser_scan_cloud_.back(), (--fragment.cend())->point());
}

INSTANTIATE_TEST_CASE_P(FragmentTestData,
                        LaserScanFragmentTestWithParam,
                        testing::Values(test::getFragment1(), test::getFragment11(), test::getFragment2(),
                                        test::getFragmentUnique1(), test::getFragmentUnique2()));
