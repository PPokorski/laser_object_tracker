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

#include "laser_object_tracker/data_association/naive_linear_assignment.hpp"

namespace laser_object_tracker {
namespace data_association {
NaiveLinearAssignment::NaiveLinearAssignment(double max_allowed_cost) : BaseDataAssociation(max_allowed_cost) {}

double NaiveLinearAssignment::solve(const Eigen::MatrixXd& cost_matrix,
                                    const Eigen::MatrixXd& covariance_matrix,
                                    Eigen::VectorXi& assignment_vector) {
  assignment_vector.setConstant(cost_matrix.rows(), NO_ASSIGNMENT);
  double assignment_cost = 0.0;
  for (int row = 0; row < cost_matrix.rows(); ++row) {
    double cost = std::numeric_limits<double>::infinity();
    int assigned_index = NO_ASSIGNMENT;
    for (int col = 0; col < cost_matrix.cols(); ++col) {
      if (cost_matrix(row, col) < cost &&
          cost_matrix(row, col) <= max_allowed_cost_ &&
          (assignment_vector.head(row).array() != col).all()) {
        cost = cost_matrix(row, col);
        assigned_index = col;
      }
    }

    assignment_vector(row) = assigned_index;
    assignment_cost += std::isfinite(cost) ? cost : 0.0;
  }

  return assignment_cost;
}
}  // namespace data_association
}  // namespace laser_object_tracker
