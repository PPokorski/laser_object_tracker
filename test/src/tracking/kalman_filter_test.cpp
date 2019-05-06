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

#include "laser_object_tracker/tracking/kalman_filter.hpp"

#include "test/utils.hpp"

TEST(KalmanFilterTest, InitFromStateTest) {
  Eigen::MatrixXd empty_matrix;
  Eigen::MatrixXd measurement_matrix(2, 4);
  measurement_matrix << 1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0;
  laser_object_tracker::tracking::KalmanFilter filter(4, 2,
      empty_matrix,
      measurement_matrix,
      empty_matrix,
      empty_matrix,
      empty_matrix);

  Eigen::VectorXd expected_state = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd state_vector = filter.getStateVector();
  EXPECT_TRUE(expected_state.isApprox(state_vector, test::PRECISION<double>))
      << "Expected state vector is:\n" << expected_state << std::endl
      << "but actual is:\n" << state_vector;

  laser_object_tracker::feature_extraction::features::Feature example_state;
  example_state.observation_.resize(4);
  example_state.observation_ << 10.0, 32.0, 53.0, 21.0;
  filter.initFromState(example_state);
  expected_state = example_state.observation_;
  state_vector = filter.getStateVector();
  EXPECT_TRUE(expected_state.isApprox(state_vector, test::PRECISION<double>))
            << "Expected state vector is:\n" << expected_state << std::endl
            << "but actual is:\n" << state_vector;
}

TEST(KalmanFilterTest, InitFromMeasurementTest) {
  Eigen::MatrixXd empty_matrix;
  Eigen::MatrixXd measurement_matrix(2, 4);
  measurement_matrix << 1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0;
  laser_object_tracker::tracking::KalmanFilter filter(4, 2,
                                                      empty_matrix,
                                                      measurement_matrix,
                                                      empty_matrix,
                                                      empty_matrix,
                                                      empty_matrix);

  laser_object_tracker::feature_extraction::features::Feature measurement;
  measurement.observation_.resize(2);
  measurement.observation_ << 21.0, 32.0;
  filter.initFromMeasurement(measurement);
  Eigen::VectorXd expected_state = Eigen::VectorXd::Zero(4);
  expected_state.head<2>() = measurement.observation_;
  Eigen::VectorXd state_vector = filter.getStateVector();
  EXPECT_TRUE(expected_state.isApprox(state_vector, test::PRECISION<double>))
            << "Expected state vector is:\n" << expected_state << std::endl
            << "but actual is:\n" << state_vector;

  measurement_matrix << 1.0, 0.0, 1.0, 0.0,
                        0.0, 2.0, 0.0, 2.0;
  laser_object_tracker::tracking::KalmanFilter filter2(4, 2,
                                                       empty_matrix,
                                                       measurement_matrix,
                                                       empty_matrix,
                                                       empty_matrix,
                                                       empty_matrix);

  measurement.observation_ << 21.0, 32.0;
  filter2.initFromMeasurement(measurement);
  expected_state << 10.5, 8.0, 10.5, 8.0;
  state_vector = filter2.getStateVector();
  EXPECT_TRUE(expected_state.isApprox(state_vector, test::PRECISION<double>))
            << "Expected state vector is:\n" << expected_state << std::endl
            << "but actual is:\n" << state_vector;
}
