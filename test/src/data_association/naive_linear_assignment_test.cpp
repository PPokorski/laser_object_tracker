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

#include "laser_object_tracker/data_association/naive_linear_assignment.hpp"

#include "test/utils.hpp"

TEST(NaiveLinearAssignmentTest, EmptyMatrixTest) {
  laser_object_tracker::data_association::NaiveLinearAssignment naive_linear_assignment;

  Eigen::MatrixXd cost_matrix;
  Eigen::VectorXi assignment_vector;

  EXPECT_NEAR(0.0,
              naive_linear_assignment.solve(cost_matrix,
                                            naive_linear_assignment.NOT_NEEDED,
                                            assignment_vector),
              test::PRECISION<double>);

  Eigen::VectorXi expected_assignment;
  EXPECT_EQ(expected_assignment, assignment_vector);
}

TEST(NaiveLinearAssignmentTest, AllAssignedTest) {
  laser_object_tracker::data_association::NaiveLinearAssignment naive_linear_assignment;

  Eigen::MatrixXd cost_matrix(4, 4);
  cost_matrix << 0.0, 1.0, 1.0, 1.0,
      1.0, 0.0, 1.0, 1.0,
      1.0, 1.0, 0.0, 1.0,
      1.0, 1.0, 1.0, 0.0;
  Eigen::VectorXi assignment_vector;
  EXPECT_NEAR(0.0,
              naive_linear_assignment.solve(cost_matrix,
                                            naive_linear_assignment.NOT_NEEDED,
                                            assignment_vector),
              test::PRECISION<double>);
  Eigen::VectorXi expected_assignment(4);
  expected_assignment << 0, 1, 2, 3;
  EXPECT_EQ(expected_assignment, assignment_vector);

  cost_matrix << 23.0, 1.0, 41.0, 87.0,
      2.0, 23.0, 324.0, 65.0,
      43.0, 424.0, 121.0, 4.0,
      122.0, 53.0, 3.0, 765.0;
  EXPECT_NEAR(10.0,
              naive_linear_assignment.solve(cost_matrix,
                                            naive_linear_assignment.NOT_NEEDED,
                                            assignment_vector),
              test::PRECISION<double>);
  expected_assignment << 1, 0, 3, 2;
  EXPECT_EQ(expected_assignment, assignment_vector);
}

TEST(NaiveLinearAssignmentTest, NoAssignmentTest) {
  laser_object_tracker::data_association::NaiveLinearAssignment naive_linear_assignment;

  Eigen::MatrixXd cost_matrix(2, 4);
  cost_matrix << 0.0, 1.0, 2.0, 3.0,
                 7.0, 2.0, 0.0, 5.0;
  Eigen::VectorXi assignment_vector;
  EXPECT_NEAR(2.0,
              naive_linear_assignment.solve(cost_matrix,
                                            naive_linear_assignment.NOT_NEEDED,
                                            assignment_vector),
              test::PRECISION<double>);
  Eigen::VectorXi expected_assignment(4);
  expected_assignment << 0, 1, naive_linear_assignment.NO_ASSIGNMENT, naive_linear_assignment.NO_ASSIGNMENT;
  EXPECT_EQ(expected_assignment, assignment_vector);
}

TEST(NaiveLinearAssignmentTest, MaxAllowedCostTest) {
  laser_object_tracker::data_association::NaiveLinearAssignment naive_linear_assignment(10.0);

  Eigen::MatrixXd cost_matrix(2, 2);
  cost_matrix << 11.0, 12.0,
                 13.0, 14.0;
  Eigen::VectorXi assignment_vector;
  EXPECT_NEAR(0.0,
              naive_linear_assignment.solve(cost_matrix,
                                            naive_linear_assignment.NOT_NEEDED,
                                            assignment_vector),
              test::PRECISION<double>);
  Eigen::VectorXi expected_assignment(2);
  expected_assignment << naive_linear_assignment.NO_ASSIGNMENT, naive_linear_assignment.NO_ASSIGNMENT;
  EXPECT_EQ(expected_assignment, assignment_vector);
}
