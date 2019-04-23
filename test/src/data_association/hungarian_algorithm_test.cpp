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

#include "laser_object_tracker/data_association/hungarian_algorithm.hpp"

#include "test/utils.hpp"

TEST(HungarianAlgorithmTest, EmptyMatrixTest) {
  laser_object_tracker::data_association::HungarianAlgorithm hungarian_algorithm;

  Eigen::MatrixXd cost_matrix;
  Eigen::VectorXi assignment_vector;

  EXPECT_NEAR(0.0,
              hungarian_algorithm.solve(cost_matrix,
                                        hungarian_algorithm.NOT_NEEDED,
                                        assignment_vector),
              test::PRECISION<double>);

  Eigen::VectorXi expected_assignment;
  EXPECT_EQ(expected_assignment, assignment_vector);
}

TEST(HungarianAlgorithmTest, AllAssignedTest) {
  laser_object_tracker::data_association::HungarianAlgorithm hungarian_algorithm;

  Eigen::MatrixXd cost_matrix(4, 4);
  cost_matrix << 0.0, 1.0, 1.0, 1.0,
                 1.0, 0.0, 1.0, 1.0,
                 1.0, 1.0, 0.0, 1.0,
                 1.0, 1.0, 1.0, 0.0;
  Eigen::VectorXi assignment_vector;
  EXPECT_NEAR(0.0,
              hungarian_algorithm.solve(cost_matrix,
                                        hungarian_algorithm.NOT_NEEDED,
                                        assignment_vector),
              test::PRECISION<double>);
  Eigen::VectorXi expected_assignment(4);
  expected_assignment << 0, 1, 2, 3;
  EXPECT_EQ(expected_assignment, assignment_vector);

  cost_matrix << 21.0, 49.0, 14.0, 45.0,
                 30.0, 21.0, 67.0,  7.0,
                 26.0, 39.0, 66.0, 72.0,
                  6.0, 40.0, 54.0, 43.0;
  EXPECT_NEAR(66.0,
              hungarian_algorithm.solve(cost_matrix,
                                        hungarian_algorithm.NOT_NEEDED,
                                        assignment_vector),
              test::PRECISION<double>);
  expected_assignment << 3, 2, 0, 1;
  EXPECT_EQ(expected_assignment, assignment_vector);

  cost_matrix.resize(5, 4);
  cost_matrix <<  2.0, 42.0, 25.0,  7.0,
                 50.0, 27.0, 39.0, 27.0,
                 50.0, 89.0, 68.0, 10.0,
                 91.0,  6.0, 76.0, 81.0,
                 21.0, 35.0, 86.0, 23.0;
  EXPECT_NEAR(57.0,
              hungarian_algorithm.solve(cost_matrix,
                                        hungarian_algorithm.NOT_NEEDED,
                                        assignment_vector),
              test::PRECISION<double>);
  expected_assignment << 0, 3, 1, 2;
  EXPECT_EQ(expected_assignment, assignment_vector);
}

TEST(HungarianAlgorithmTest, NoAssignmentTest) {
  laser_object_tracker::data_association::HungarianAlgorithm hungarian_algorithm;

  Eigen::MatrixXd cost_matrix(3, 4);
  cost_matrix << 20.0, 81.0, 44.0,  9.0,
                 85.0,  3.0, 11.0, 93.0,
                 29.0,  3.0, 39.0, 47.0;
  Eigen::VectorXi assignment_vector;
  EXPECT_NEAR(23.0,
              hungarian_algorithm.solve(cost_matrix,
                                        hungarian_algorithm.NOT_NEEDED,
                                        assignment_vector),
              test::PRECISION<double>);
  Eigen::VectorXi expected_assignment(4);
  expected_assignment << hungarian_algorithm.NO_ASSIGNMENT, 2, 1, 0;
  EXPECT_EQ(expected_assignment, assignment_vector);
}

TEST(HungarianAlgorithmTest, MaxAllowedCostTest) {
  laser_object_tracker::data_association::HungarianAlgorithm hungarian_algorithm(10.0);

  Eigen::MatrixXd cost_matrix(4, 4);
  cost_matrix << 21.0,  3.0, 26.0,  6.0,
                 49.0, 21.0, 39.0, 40.0,
                 14.0, 67.0, 66.0, 54.0,
                 45.0,  7.0, 72.0, 43.0;
  Eigen::VectorXi assignment_vector;
  EXPECT_NEAR(13.0,
              hungarian_algorithm.solve(cost_matrix,
                                        hungarian_algorithm.NOT_NEEDED,
                                        assignment_vector),
              test::PRECISION<double>);
  Eigen::VectorXi expected_assignment(4);
  expected_assignment << hungarian_algorithm.NO_ASSIGNMENT, 3, hungarian_algorithm.NO_ASSIGNMENT, 0;
  EXPECT_EQ(expected_assignment, assignment_vector);
}
