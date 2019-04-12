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

#include "laser_object_tracker/feature_extraction/pcl/sac_model_cross2d.hpp"

#include "test/utils.hpp"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Model = pcl::SampleConsenusModelCross2D<pcl::PointXYZ>;

TEST(SampleConsensusModelCross2DTest, ModelValidityTest) {
    PointCloud cloud;
    cloud.push_back({0.0, 0.0, 0.0});
    Model model(cloud.makeShared());

    std::vector<int> samples;
    Eigen::VectorXf model_coefficients;
    ASSERT_FALSE(model.computeModelCoefficients(samples, model_coefficients));

    samples.push_back(0);
    ASSERT_FALSE(model.computeModelCoefficients(samples, model_coefficients));

    samples.push_back(0);
    ASSERT_FALSE(model.computeModelCoefficients(samples, model_coefficients));

    samples.push_back(0);
    ASSERT_TRUE(model.computeModelCoefficients(samples, model_coefficients));

    samples.push_back(0);
    ASSERT_FALSE(model.computeModelCoefficients(samples, model_coefficients));
}

TEST(SampleConsensusModelCross2DTest, ComputeModelCoefficientsTest) {
    PointCloud cloud;
    cloud.points = {
            {0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0},
            {2.0, 2.0, 0.0},
            {3.0, 1.0, 0.0},
            {4.0, 0.0, 0.0}
    };
    Model model(cloud.makeShared());

    std::vector<int> samples {0, 2, 4};
    Eigen::VectorXf model_coefficients;
    ASSERT_TRUE(model.computeModelCoefficients(samples, model_coefficients));
    ASSERT_EQ(4, model_coefficients.size());

    Eigen::VectorXf expected_coefficients(4);
    expected_coefficients << 2.0, 2.0, -M_SQRT1_2, -M_SQRT1_2;
    EXPECT_TRUE(model_coefficients.isApprox(expected_coefficients, test::PRECISION<double>))
            << "Expected coefficients are\n" << expected_coefficients.transpose()
            << "\nbut actual are\n" << model_coefficients.transpose();

    samples = {1, 2, 3};
    ASSERT_TRUE(model.computeModelCoefficients(samples, model_coefficients));
    EXPECT_TRUE(model_coefficients.isApprox(expected_coefficients, test::PRECISION<double>))
                        << "Expected coefficients are\n" << expected_coefficients.transpose()
                        << "\nbut actual are\n" << model_coefficients.transpose();

    samples = {2, 1, 3};
    ASSERT_TRUE(model.computeModelCoefficients(samples, model_coefficients));
    EXPECT_TRUE(model_coefficients.isApprox(expected_coefficients, test::PRECISION<double>))
                        << "Expected coefficients are\n" << expected_coefficients.transpose()
                        << "\nbut actual are\n" << model_coefficients.transpose();

    samples = {4, 1, 2};
    ASSERT_TRUE(model.computeModelCoefficients(samples, model_coefficients));
    EXPECT_TRUE(model_coefficients.isApprox(expected_coefficients, test::PRECISION<double>))
                        << "Expected coefficients are\n" << expected_coefficients.transpose()
                        << "\nbut actual are\n" << model_coefficients.transpose();

    samples = {2, 3, 0};
    ASSERT_TRUE(model.computeModelCoefficients(samples, model_coefficients));
    EXPECT_TRUE(model_coefficients.isApprox(expected_coefficients, test::PRECISION<double>))
                        << "Expected coefficients are\n" << expected_coefficients.transpose()
                        << "\nbut actual are\n" << model_coefficients.transpose();

    cloud.points = {
            {1.0, 0.0, 0.0},
            {1.0, -1.0, 0.0},
            {1.0, -2.0, 0.0},
            {0.0, -2.0, 0.0}
    };
    model.setInputCloud(cloud.makeShared());

    samples = {0, 2, 3};
    expected_coefficients << 1.0, -2.0, 0, 1.0;
    ASSERT_TRUE(model.computeModelCoefficients(samples, model_coefficients));
    EXPECT_TRUE(model_coefficients.isApprox(expected_coefficients, test::PRECISION<double>))
                        << "Expected coefficients are\n" << expected_coefficients.transpose()
                        << "\nbut actual are\n" << model_coefficients.transpose();

    samples = {3, 1, 2};
    ASSERT_TRUE(model.computeModelCoefficients(samples, model_coefficients));
    EXPECT_TRUE(model_coefficients.isApprox(expected_coefficients, test::PRECISION<double>))
                        << "Expected coefficients are\n" << expected_coefficients.transpose()
                        << "\nbut actual are\n" << model_coefficients.transpose();
}

TEST(SampleConsensusModelCross2DTest, OptimizeModelCoefficientsTest) {
    PointCloud cloud;
    cloud.points = {
            {0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0},
            {2.0, 2.0, 0.0},
            {3.0, 1.0, 0.0},
            {4.0, 0.0, 0.0}
    };
    Model model(cloud.makeShared());
    std::vector<int> inliers {0, 1, 2, 3, 4};
    Eigen::VectorXf model_coefficients(4);
    model_coefficients << 2.0, 2.0, -M_SQRT1_2, -M_SQRT1_2;
    Eigen::VectorXf optimized_coefficients;
    model.optimizeModelCoefficients(inliers, model_coefficients, optimized_coefficients);

    Eigen::VectorXf expected_coefficients(4);
    expected_coefficients << 2.0, 2.0, -M_SQRT1_2, -M_SQRT1_2;
    EXPECT_TRUE(optimized_coefficients.isApprox(expected_coefficients, test::PRECISION<double>))
                        << "Expected coefficients are\n" << expected_coefficients.transpose()
                        << "\nbut actual are\n" << optimized_coefficients.transpose();

    model_coefficients << 1.9, 2.0, -M_SQRT1_2, -M_SQRT1_2;
    model.optimizeModelCoefficients(inliers, model_coefficients, optimized_coefficients);
    EXPECT_TRUE(optimized_coefficients.isApprox(expected_coefficients, 0.01))
                        << "Expected coefficients are\n" << expected_coefficients.transpose()
                        << "\nbut actual are\n" << optimized_coefficients.transpose();

    model_coefficients << 1.9, 2.1, -M_SQRT1_2 + 0.01, -M_SQRT1_2 - 0.01;
    model.optimizeModelCoefficients(inliers, model_coefficients, optimized_coefficients);
    EXPECT_TRUE(optimized_coefficients.isApprox(expected_coefficients, 0.01))
                        << "Expected coefficients are\n" << expected_coefficients.transpose()
                        << "\nbut actual are\n" << optimized_coefficients.transpose();
}

TEST(SampleConsensusModelCross2DTest, GetDistanceToModelTest) {
    PointCloud cloud;
    cloud.points = {
            {0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0},
            {2.0, 2.0, 0.0},
            {3.0, 3.0, 0.0},
            {12.0, 37.0, 0.0},
            {-10.0, -5.0, 0.0},
            {25.0, -13.0, 0.0}
    };
    Model model(cloud.makeShared());

    using namespace std::placeholders;
    auto compare_function = [](double x, double y) {
            return std::abs(x - y) <= test::PRECISION<double>;
        };

    Eigen::VectorXf model_coefficients(4);
    model_coefficients << 0.0, 0.0, 1.0, 0.0;
    std::vector<double> distances;
    model.getDistancesToModel(model_coefficients, distances);

    ASSERT_EQ(cloud.points.size(), distances.size());
    std::vector<double> expected_distances {0.0, 1.0, 2.0, 3.0, 12.0, 5.0, 13.0};
    for (int i = 0; i < expected_distances.size(); ++i)
    {
        EXPECT_NEAR(expected_distances.at(i), distances.at(i), test::PRECISION<double>);
    }

    model_coefficients << 3.0, 2.0, 2.0, 1.0;
    model.getDistancesToModel(model_coefficients, distances);

    expected_distances = {0.4472135955, 0.0, 0.4472135955, 0.4472135955, 23.702320561, 0.4472135955, 12.9691942694};
    for (int i = 0; i < expected_distances.size(); ++i)
    {
        EXPECT_NEAR(expected_distances.at(i), distances.at(i), test::PRECISION<double>);
    }
}

TEST(SampleConsensusModelCross2DTest, SelectWithinDistanceTest) {
    PointCloud cloud;
    cloud.points = {
            {0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0},
            {2.0, 2.0, 0.0},
            {3.0, 3.0, 0.0},
            {12.0, 37.0, 0.0},
            {-10.0, -5.0, 0.0},
            {25.0, -13.0, 0.0}
    };
    Model model(cloud.makeShared());

    Eigen::VectorXf model_coefficients(4);
    model_coefficients << 0.0, 0.0, 1.0, 0.0;
    std::vector<int> inliers;
    model.selectWithinDistance(model_coefficients, 10.0, inliers);

    std::vector<int> expected_inliers {0, 1, 2, 3, 5};
    EXPECT_EQ(expected_inliers, inliers);

    model.selectWithinDistance(model_coefficients, 10000.0, inliers);
    expected_inliers = {0, 1, 2, 3, 4, 5, 6};
    EXPECT_EQ(expected_inliers, inliers);

    model_coefficients << 3.0, 2.0, 2.0, 1.0;
    model.selectWithinDistance(model_coefficients, 0.3, inliers);
    expected_inliers = {1};
    EXPECT_EQ(expected_inliers, inliers);

    model.selectWithinDistance(model_coefficients, 0.5, inliers);
    expected_inliers = {0, 1, 2, 3, 5};
    EXPECT_EQ(expected_inliers, inliers);
}

TEST(SampleConsensusModelCross2DTest, CountWithinDistanceTest) {
    PointCloud cloud;
    cloud.points = {
            {0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0},
            {2.0, 2.0, 0.0},
            {3.0, 3.0, 0.0},
            {12.0, 37.0, 0.0},
            {-10.0, -5.0, 0.0},
            {25.0, -13.0, 0.0}
    };
    Model model(cloud.makeShared());

    Eigen::VectorXf model_coefficients(4);
    model_coefficients << 0.0, 0.0, 1.0, 0.0;
    std::vector<int> inliers;
    EXPECT_EQ(5, model.countWithinDistance(model_coefficients, 10.0));

    EXPECT_EQ(7, model.countWithinDistance(model_coefficients, 10000.0));

    model_coefficients << 3.0, 2.0, 2.0, 1.0;
    EXPECT_EQ(1, model.countWithinDistance(model_coefficients, 0.3));

    EXPECT_EQ(5, model.countWithinDistance(model_coefficients, 0.5));
}

TEST(SampleConsensusModelCross2DTest, ProjectPointsTest) {
    PointCloud cloud;
    cloud.points = {
            {0.0, 0.0, 0.0},
            {1.0, 1.5, 0.0},
            {2.5, 2.0, 0.0},
            {3.0, 3.5, 0.0},
            {12.0, 37.0, 0.0},
            {-10.0, -5.0, 0.0},
            {25.0, -13.0, 0.0}
    };
    Model model(cloud.makeShared());

    Eigen::VectorXf model_coefficients(4);
    model_coefficients << 0.0, 0.0, 1.0, 0.0;
    std::vector<int> inliers {0, 1, 2, 3, 4, 5, 6};

    PointCloud projected_cloud;
    model.projectPoints(inliers, model_coefficients, projected_cloud, true);

    PointCloud expected_cloud;
    expected_cloud.points = {
            {0.0, 0.0, 0.0},
            {0.0, 1.5, 0.0},
            {2.5, 0.0, 0.0},
            {0.0, 3.5, 0.0},
            {0.0, 37.0, 0.0},
            {-10.0, 0.0, 0.0},
            {25.0, 0.0, 0.0}
    };
    auto compare_function = [](const PointCloud& lhs, const PointCloud& rhs) {
        return test::compare(lhs, rhs);
    };
    EXPECT_PRED2(compare_function, expected_cloud, projected_cloud);

    projected_cloud.clear();
    model.projectPoints(inliers, model_coefficients, projected_cloud, false);
    EXPECT_PRED2(compare_function, expected_cloud, projected_cloud);

    inliers = {0, 1, 3, 5};
    model.projectPoints(inliers, model_coefficients, projected_cloud, true);
    expected_cloud.points = {
            {0.0, 0.0, 0.0},
            {0.0, 1.5, 0.0},
            {2.5, 2.0, 0.0},
            {0.0, 3.5, 0.0},
            {12.0, 37.0, 0.0},
            {-10.0, 0.0, 0.0},
            {25.0, -13.0, 0.0}
    };
    EXPECT_PRED2(compare_function, expected_cloud, projected_cloud);

    projected_cloud.clear();
    model.projectPoints(inliers, model_coefficients, projected_cloud, false);
    expected_cloud.points = {
            {0.0, 0.0, 0.0},
            {0.0, 1.5, 0.0},
            {0.0, 3.5, 0.0},
            {-10.0, 0.0, 0.0},
    };
    expected_cloud.width = 4;
    expected_cloud.height = 1;
    EXPECT_PRED2(compare_function, expected_cloud, projected_cloud);

    model_coefficients << 3.0, 2.0, 2.0, 1.0;
    inliers = {0, 1, 2, 3, 4, 5, 6};
    projected_cloud.clear();
    model.projectPoints(inliers, model_coefficients, projected_cloud, false);
    expected_cloud.points = {
            {-0.2, 0.4, 0.0},
            {1.2, 1.1, 0.0},
            {2.6, 1.8, 0.0},
            {2.4, 3.2, 0.0},
            {-9.2, 26.4, 0.0},
            {-10.2, -4.6, 0.0},
            {13.4, -18.8, 0.0}
    };
    expected_cloud.width = 7;
    expected_cloud.height = 1;
    EXPECT_PRED2(compare_function, expected_cloud, projected_cloud);
}

TEST(SampleConsensusModelCross2DTest, DoSamplesVerifyModelTest) {
    PointCloud cloud;
    cloud.points = {
            {0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0},
            {2.0, 2.0, 0.0},
            {3.0, 3.0, 0.0},
            {12.0, 37.0, 0.0},
            {-10.0, -5.0, 0.0},
            {25.0, -13.0, 0.0}
    };
    Model model(cloud.makeShared());

    Eigen::VectorXf model_coefficients(4);
    model_coefficients << 0.0, 0.0, 1.0, 0.0;
    std::set<int> indices {0, 1};
    EXPECT_TRUE(model.doSamplesVerifyModel(indices, model_coefficients, 1.5));
    EXPECT_FALSE(model.doSamplesVerifyModel(indices, model_coefficients, 0.9));

    indices = {0, 1, 2, 3, 4, 5, 6};
    EXPECT_TRUE(model.doSamplesVerifyModel(indices, model_coefficients, 15.0));
    EXPECT_FALSE(model.doSamplesVerifyModel(indices, model_coefficients, 12.9));

    model_coefficients << 3.0, 2.0, 2.0, 1.0;
    indices = {0, 1, 2, 3, 5};
    EXPECT_TRUE(model.doSamplesVerifyModel(indices, model_coefficients, 0.5));
    EXPECT_FALSE(model.doSamplesVerifyModel(indices, model_coefficients, 0.4));
}

