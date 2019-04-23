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

#include "laser_object_tracker/tracking/multi_tracker.hpp"

namespace laser_object_tracker {
namespace tracking {
MultiTracker::MultiTracker(DistanceFunctor distance_calculator,
                           std::unique_ptr<data_association::BaseDataAssociation> data_association,
                           std::unique_ptr<BaseTracking> tracker_prototype)
    : distance_calculator_(std::move(distance_calculator)),
      data_association_(std::move(data_association)),
      tracker_prototype_(std::move(tracker_prototype)) {}

void MultiTracker::predict() {
  for (auto& tracker : trackers_) {
    tracker->predict();
  }
}

void MultiTracker::update(const std::vector<Eigen::VectorXd>& measurements) {
  Eigen::MatrixXd cost_matrix(trackers_.size(), measurements.size());
  for (int row = 0; row < cost_matrix.rows(); ++row) {
    for (int col = 0; col < cost_matrix.cols(); ++col) {
      cost_matrix(row, col) = distance_calculator_(measurements.at(col), *trackers_.at(row));
    }
  }

  Eigen::VectorXi assignment_vector;
  data_association_->solve(cost_matrix, data_association_->NOT_NEEDED, assignment_vector);

  for (int i = 0; i < measurements.size(); ++i) {
    if (assignment_vector(i) != data_association_->NO_ASSIGNMENT) {
      trackers_.at(assignment_vector(i))->update(measurements.at(i));
    } else {
      trackers_.push_back(std::move(tracker_prototype_->clone()));
      trackers_.back()->initFromMeasurement(measurements.at(i));
    }
  }
}
}  // namespace tracking
}  // namespace laser_object_tracker
