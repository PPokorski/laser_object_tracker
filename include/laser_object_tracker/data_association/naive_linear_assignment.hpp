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

#ifndef LASER_OBJECT_TRACKER_DATA_ASSOCIATION_NAIVE_LINEAR_ASSIGNMENT_HPP
#define LASER_OBJECT_TRACKER_DATA_ASSOCIATION_NAIVE_LINEAR_ASSIGNMENT_HPP

#include "laser_object_tracker/data_association/base_data_association.hpp"

namespace laser_object_tracker {
namespace data_association {

class NaiveLinearAssignment : public BaseDataAssociation {
 public:
  explicit NaiveLinearAssignment(double max_allowed_cost = std::numeric_limits<double>::infinity());

  double solve(const Eigen::MatrixXd& cost_matrix,
               const Eigen::MatrixXd& covariance_matrix,
               Eigen::VectorXi& assignment_vector) override;
};
}  // namespace data_association
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_DATA_ASSOCIATION_NAIVE_LINEAR_ASSIGNMENT_HPP
