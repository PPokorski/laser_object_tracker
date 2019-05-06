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

#include <numeric>

namespace laser_object_tracker {
namespace tracking {
MultiTracker::MultiTracker(DistanceFunctor distance_calculator,
                           std::unique_ptr<data_association::BaseDataAssociation> data_association,
                           std::unique_ptr<BaseTracking> tracker_prototype,
                           std::unique_ptr<BaseTrackerRejection> tracker_rejector_prototype)
    : distance_calculator_(std::move(distance_calculator)),
      data_association_(std::move(data_association)),
      tracker_prototype_(std::move(tracker_prototype)),
      tracker_rejector_prototype_(std::move(tracker_rejector_prototype)) {}

void MultiTracker::predict() {
  for (auto& tracker : trackers_) {
    tracker->predict();
  }
}

void MultiTracker::update(const std::vector<feature_extraction::features::Feature>& measurements) {
  Eigen::MatrixXd cost_matrix = buildCostMatrix(measurements);

  Eigen::VectorXi assignment_vector = buildAssignmentVector(cost_matrix);

  updateAndInitializeTracks(measurements, assignment_vector);

  handleNotUpdatedTracks(assignment_vector);

  handleRejectedTracks();
}

std::vector<std::unique_ptr<BaseTracking>>::const_iterator MultiTracker::begin() const {
  return trackers_.begin();
}

std::vector<std::unique_ptr<BaseTracking>>::const_iterator MultiTracker::end() const {
  return trackers_.end();
}

std::vector<std::unique_ptr<BaseTracking>>::const_iterator MultiTracker::cbegin() const {
  return trackers_.cbegin();
}

std::vector<std::unique_ptr<BaseTracking>>::const_iterator MultiTracker::cend() const {
  return trackers_.cend();
}

const BaseTracking& MultiTracker::at(int index) const {
  return *trackers_.at(index);
}

int MultiTracker::size() const {
  return trackers_.size();
}

Eigen::MatrixXd MultiTracker::buildCostMatrix(const std::vector<feature_extraction::features::Feature>& measurements) {
  Eigen::MatrixXd cost_matrix(trackers_.size(), measurements.size());
  for (int row = 0; row < cost_matrix.rows(); ++row) {
    for (int col = 0; col < cost_matrix.cols(); ++col) {
      cost_matrix(row, col) = distance_calculator_(measurements.at(col), *trackers_.at(row));
    }
  }

  return cost_matrix;
}

Eigen::VectorXi MultiTracker::buildAssignmentVector(const Eigen::MatrixXd& cost_matrix) {
  Eigen::VectorXi assignment_vector;
  data_association_->solve(cost_matrix, data_association_->NOT_NEEDED, assignment_vector);

  return assignment_vector;
}

void MultiTracker::updateAndInitializeTracks(const std::vector<feature_extraction::features::Feature>& measurements,
                                             const Eigen::VectorXi& assignment_vector) {
  for (int i = 0; i < measurements.size(); ++i) {
    if (assignment_vector(i) != data_association_->NO_ASSIGNMENT) {
      int tracker_index = assignment_vector(i);
      trackers_.at(tracker_index)->update(measurements.at(i));
      trackers_rejections_.at(tracker_index)->updated(*trackers_.at(tracker_index));
    } else {
      trackers_.push_back(std::move(tracker_prototype_->clone()));
      trackers_.back()->initFromMeasurement(measurements.at(i));
      trackers_rejections_.push_back(std::move(tracker_rejector_prototype_->clone()));
    }
  }
}

void MultiTracker::handleNotUpdatedTracks(const Eigen::VectorXi& assignment_vector) {
  std::vector<int> trackers_indices(trackers_.size());
  std::iota(trackers_indices.begin(), trackers_indices.end(), 0);

  std::vector<int> updated_trackers(assignment_vector.data(), assignment_vector.data() + assignment_vector.size());
  std::sort(updated_trackers.begin(), updated_trackers.end());

  std::vector<int> not_updated_trackers;
  not_updated_trackers.reserve(trackers_indices.size());
  std::set_difference(trackers_indices.begin(),
                      trackers_indices.end(),
                      updated_trackers.begin(),
                      updated_trackers.end(),
                      std::back_inserter(not_updated_trackers));

  for (int index : not_updated_trackers) {
    trackers_rejections_.at(index)->notUpdated(*trackers_.at(index));
  }
}

void MultiTracker::handleRejectedTracks() {
  auto trackers_it = trackers_.cbegin();
  auto rejectors_it = trackers_rejections_.cbegin();
  for (; trackers_it != trackers_.cend(); ) {
    const auto& tracker = **trackers_it;
    const auto& rejector = **rejectors_it;
    if (rejector.invalidate(tracker)) {
      trackers_it = trackers_.erase(trackers_it);
      rejectors_it = trackers_rejections_.erase(rejectors_it);
    } else {
      ++trackers_it;
      ++rejectors_it;
    }
  }
}
}  // namespace tracking
}  // namespace laser_object_tracker
