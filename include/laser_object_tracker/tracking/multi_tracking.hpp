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

#ifndef LASER_OBJECT_TRACKER_TRACKING_MULTI_TRACKING_H
#define LASER_OBJECT_TRACKER_TRACKING_MULTI_TRACKING_H

#include <vector>

#include "laser_object_tracker/data_association/base_data_association.hpp"
#include "laser_object_tracker/tracking/base_multi_tracking.hpp"
#include "laser_object_tracker/tracking/base_tracker_rejection.hpp"
#include "laser_object_tracker/tracking/base_tracking.hpp"

namespace laser_object_tracker {
namespace tracking {
struct TrackElement {
  double likelihood_;
  ros::Time timestamp_;

  Eigen::Vector2d position_;
  Eigen::Matrix2d position_covariance_;

  Eigen::Vector2d velocity_;
  Eigen::Matrix2d velocity_covariance_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Track {
  Track() = default;

  explicit Track(int id) : id_(id) {}

  Track(int id, std::vector<TrackElement, Eigen::aligned_allocator<TrackElement>> track)
      : id_(id),
        track_(std::move(track)) {}

  int id_;

  std::vector<TrackElement, Eigen::aligned_allocator<TrackElement>> track_;
};

class MultiTracking : public BaseMultiTracking<feature_extraction::features::Feature, Track> {
 public:
  using DistanceFunctor = std::function<double(const FeatureT&, const BaseTracking&)>;

  MultiTracking(DistanceFunctor distance_calculator,
               std::unique_ptr<data_association::BaseDataAssociation> data_association,
               std::unique_ptr<BaseTracking> tracker_prototype,
               std::unique_ptr<BaseTrackerRejection> tracker_rejector_prototype);

  void predict() override;

  const Container& update(const std::vector<FeatureT>& measurements) override;

  Eigen::MatrixXd buildCostMatrix(const std::vector<FeatureT>& measurements);

  Eigen::VectorXi buildAssignmentVector(const Eigen::MatrixXd& cost_matrix);

  void updateAndInitializeTracks(const std::vector<FeatureT>& measurements,
                                 const Eigen::VectorXi& assignment_vector);

  void handleNotUpdatedTracks(const Eigen::VectorXi& assignment_vector);

  void handleRejectedTracks();

  std::vector<std::unique_ptr<BaseTracking>>::const_iterator begin() const;

  std::vector<std::unique_ptr<BaseTracking>>::const_iterator end() const;

  std::vector<std::unique_ptr<BaseTracking>>::const_iterator cbegin() const;

  std::vector<std::unique_ptr<BaseTracking>>::const_iterator cend() const;

  const BaseTracking& at(int index) const;

  int size() const;

 private:
  DistanceFunctor distance_calculator_;
  std::unique_ptr<data_association::BaseDataAssociation> data_association_;

  std::unique_ptr<BaseTracking> tracker_prototype_;
  std::vector<std::unique_ptr<BaseTracking>> trackers_;

  std::unique_ptr<BaseTrackerRejection> tracker_rejector_prototype_;
  std::vector<std::unique_ptr<BaseTrackerRejection>> trackers_rejections_;
};
}  // namespace tracking
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_TRACKING_MULTI_TRACKING_H
