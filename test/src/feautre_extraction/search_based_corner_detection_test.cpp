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

#include "laser_object_tracker/feature_extraction/search_based_corner_detection.hpp"

#include "test/utils.hpp"

TEST(SearchBasedCornerDetectionTest, AreaCriterionTest) {
    using namespace laser_object_tracker::feature_extraction;
    Eigen::VectorXd vector;
    EXPECT_NEAR(0.0, areaCriterion(vector, vector), test::PRECISION<double>);

    vector.resize(6);
    vector << 0.0, 1.0, 3.0, -5.0, -7.3, 3.5;
    EXPECT_NEAR(-116.64, areaCriterion(vector, vector), test::PRECISION<double>);

    Eigen::VectorXd vector2;
    vector2.resize(6);
    vector2 << -1.0, 32.0, 3.0, -21.5, 43.0, 0.02;
    EXPECT_NEAR(-696.6, areaCriterion(vector, vector2), test::PRECISION<double>);
}

TEST(SearchBasedCornerDetectionTest, ClosenessCriterionTest) {
    using namespace laser_object_tracker::feature_extraction;
    Eigen::VectorXd vector;
    EXPECT_NEAR(0.0, closenessCriterion(vector, vector), test::PRECISION<double>);

    vector.resize(5);
    vector << -10.0, -3.2, 4.3, 41.0, 34.3;
    EXPECT_NEAR(100.2591701, closenessCriterion(vector, vector), test::PRECISION<double>);

    // 0,023696682 + 100 + 0,031055901 + 0,138888889 + 0,081967213
    vector << -10.0, 32.2, 0.0, 25.0, 20.0;
    EXPECT_NEAR(100.275608685, closenessCriterion(vector, vector), test::PRECISION<double>);

    Eigen::VectorXd vector2(5);
    // 100 + 0,055865922 + 0,010467916 + 0,025445293 + 0,037355248
    vector2 << -39.3, -21.4, 56.23, 0.0, -12.53;
    EXPECT_NEAR(200.251912003, closenessCriterion(vector, vector2), test::PRECISION<double>);
}

TEST(SearchBasedCornerDetectionTest, VarianceCriterionTest) {
    using namespace laser_object_tracker::feature_extraction;
    Eigen::VectorXd vector;
    EXPECT_NEAR(0.0, varianceCriterion(vector, vector), test::PRECISION<double>);

    vector.resize(5);
    // 11.2 52.4 21.8 0 32.2
    vector << -21.0, 20.2, -10.4, -32.2, 0.0;
    EXPECT_NEAR(0.0, varianceCriterion(vector, vector), test::PRECISION<double>);

    Eigen::VectorXd vector2;
    vector2.resize(5);
    // 9.9 0.0 0.65 38.08 41.2
    vector2 << 31.3, 41.2, 40.55, 3.12, 0.0;
    EXPECT_NEAR(-259.21 -20.4438888889, varianceCriterion(vector, vector2), test::PRECISION<double>);
}

TEST(SearchBasedDetectionTest, AccessorsTest) {
    using namespace laser_object_tracker::feature_extraction;
    SearchBasedCornerDetection detection(1.0, areaCriterion);

    EXPECT_NEAR(1.0, detection.getThetaResolution(), test::PRECISION<double>);
    EXPECT_EQ(areaCriterion,
              *detection.getCriterion().target<double(*)(const Eigen::VectorXd&, const Eigen::VectorXd&)>());

    detection.setThetaResolution(0.3);
    EXPECT_NEAR(0.3, detection.getThetaResolution(), test::PRECISION<double>);

    detection.setCriterion(closenessCriterion);
    EXPECT_EQ(closenessCriterion,
              *detection.getCriterion().target<double(*)(const Eigen::VectorXd&, const Eigen::VectorXd&)>());
}

TEST(SearchBasedCornerDetectionTest, DetectionSimpleCriterionTest) {
    using namespace laser_object_tracker::data_types;
    using namespace laser_object_tracker::feature_extraction;
    SearchBasedCornerDetection detection(0.1, [](const Eigen::VectorXd&, const Eigen::VectorXd&) {
        return 0.0;
    });

    LaserScanFragment::LaserScanFragmentFactory factory;
    auto fragment = factory.fromLaserScan(test::generateLaserScan({std::sqrt(2.0f), 1.0, std::sqrt(2.0f), 1.0},
            -M_PI_4, M_PI_2));

    Eigen::VectorXd feature;
    ASSERT_TRUE(detection.extractFeature(fragment, feature));

    Eigen::VectorXd expected_feature(6);
    expected_feature << 1.0, 1.0,
                        1.0, -1.0,
                        0.0, 1.0;
    EXPECT_TRUE(expected_feature.isApprox(feature, test::PRECISION<double>))
                        << "Expected feature is\n" << expected_feature.transpose()
                        << "\n but actual is\n" << feature.transpose();
}

TEST(SearchBasedCornerDetectionTest, DetectionClosenesssCriterionTest) {
    using namespace laser_object_tracker::data_types;
    using namespace laser_object_tracker::feature_extraction;
    SearchBasedCornerDetection detection(0.001, closenessCriterion);

    LaserScanFragment::LaserScanFragmentFactory factory;
    auto fragment = factory.fromLaserScan(test::generateLaserScan({1.0, 0.73205080756, 0.73205080756, 1.0},
            -M_PI_2, M_PI_2));

    Eigen::VectorXd feature;
    ASSERT_TRUE(detection.extractFeature(fragment, feature));

    Eigen::VectorXd expected_feature(6);
    expected_feature << 1.0, 0.0,
                        0.0, -1.0,
                        0.0, 1.0;
    EXPECT_TRUE(expected_feature.isApprox(feature, 0.02))
                        << "Expected feature is\n" << expected_feature.transpose()
                        << "\n but actual is\n" << feature.transpose();
}

TEST(SearchBasedCornerDetectionTest, ExceptionThrowTest) {
    using namespace laser_object_tracker::data_types;
    using namespace laser_object_tracker::feature_extraction;
    SearchBasedCornerDetection detection(0.0, areaCriterion);

    Eigen::VectorXd feature;
    EXPECT_THROW(detection.extractFeature(LaserScanFragment(), feature), std::invalid_argument);
}
