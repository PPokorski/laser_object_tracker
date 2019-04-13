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

#include "laser_object_tracker/feature_extraction/random_sample_consensus_segment_detection.hpp"

#include "test/utils.hpp"

TEST(RandomSampleConsensusSegmentDetectionTest, AccessorsTest) {
  using namespace laser_object_tracker::feature_extraction;
  RandomSampleConsensusSegmentDetection detection(100.0, 21, 0.3);

  EXPECT_NEAR(100.0, detection.getDistanceThreshold(), test::PRECISION<double>);
  EXPECT_EQ(21, detection.getMaxIterations());
  EXPECT_NEAR(0.3, detection.getProbability(), test::PRECISION<double>);

  detection.setDistanceThreshold(20.0);
  EXPECT_NEAR(20.0, detection.getDistanceThreshold(), test::PRECISION<double>);

  detection.setMaxIterations(13);
  EXPECT_EQ(13, detection.getMaxIterations());

  detection.setProbability(0.2);
  EXPECT_NEAR(0.2, detection.getProbability(), test::PRECISION<double>);
}

TEST(RandomSampleConsensusSegmentDetectionTest, SimpleLineTest) {
  using namespace laser_object_tracker::data_types;
  using namespace laser_object_tracker::feature_extraction;
  RandomSampleConsensusSegmentDetection detection(100000.0, 100, 0.99);

  LaserScanFragment::LaserScanFragmentFactory factory;
  auto fragment = factory.fromLaserScan(test::generateLaserScan({1.0, 1.0}, -M_PI_2, M_PI_2));
  Eigen::VectorXd feature;
  ASSERT_TRUE(detection.extractFeature(fragment, feature));

  features::Point2D expected_point;
  Eigen::VectorXd expected_feature(4);
  expected_feature << 0.0, -1.0,
      0.0, 1.0;
  EXPECT_TRUE(expected_feature.isApprox(feature, test::PRECISION<double>))
            << "Expected feature is\n" << expected_feature.transpose()
            << "\n but actual is\n" << feature.transpose();

  fragment = factory.fromLaserScan(test::generateLaserScan({M_SQRT2, 1.0, M_SQRT2}, -M_PI_4, M_PI_4));
  ASSERT_TRUE(detection.extractFeature(fragment, feature));

  expected_feature << 1.0, -1.0,
      1.0, 1.0;
  EXPECT_TRUE(expected_feature.isApprox(feature, test::PRECISION<double>))
            << "Expected feature is\n" << expected_feature.transpose()
            << "\n but actual is\n" << feature.transpose();
}

TEST(RandomSampleConsensusSegmentDetectionTest, ExceptionThrowTest) {
  using namespace laser_object_tracker::data_types;
  using namespace laser_object_tracker::feature_extraction;
  RandomSampleConsensusSegmentDetection detection(0.0, 0, 0);

  Eigen::VectorXd feature;
  EXPECT_THROW(detection.extractFeature(LaserScanFragment(), feature), std::invalid_argument);
}
