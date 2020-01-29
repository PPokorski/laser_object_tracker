/*********************************************************************
*
* BSD 3-Clause License
*
*  Copyright (c) 2020, Piotr Pokorski
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

#ifndef LASER_OBJECT_TRACKER_TRACK_UNIFYING_HPP
#define LASER_OBJECT_TRACKER_TRACK_UNIFYING_HPP

#include <map>

#include <boost/bimap.hpp>

#include "laser_object_tracker/tracking/multi_hypothesis_tracking.hpp"

namespace laser_object_tracker {
namespace track_unifying {

class TrackUnifying {
 public:
  using Track = tracking::MultiHypothesisTracking::value_type;
  using Tracks = tracking::MultiHypothesisTracking::Container;

  TrackUnifying(double angle_threshold, double distance_threshold);

  std::vector<tracking::ObjectTrack> unifyTracks(const std::map<int, Tracks>& tracks);

 private:
  struct TrackSource {
    friend bool operator<(const TrackSource& lhs, const TrackSource& rhs) {
      return std::make_tuple(lhs.tracker_id, lhs.track_id) < std::make_tuple(rhs.tracker_id, rhs.track_id);
    }

    int tracker_id;
    int track_id;
  };

  struct TrackIDWithSource {
    int id;
    TrackSource source;
  };

  struct CompareId {
    using is_transparent = void;

    bool operator()(const TrackIDWithSource& lhs, const TrackIDWithSource& rhs) const {
      return lhs.id < rhs.id;
    }

    bool operator()(int lhs, const TrackIDWithSource& rhs) const {
      return lhs < rhs.id;
    }

    bool operator()(const TrackIDWithSource& lhs, int rhs) const {
      return lhs.id < rhs;
    }

    bool operator()(const TrackSource& lhs, const TrackIDWithSource& rhs) const {
      return lhs < rhs.source;
    }

    bool operator()(const TrackIDWithSource& lhs, const TrackSource& rhs) const {
      return lhs.source < rhs;
    }
  };

  void unifySourcePair(const std::pair<int, Tracks>& lhs,
                       const std::pair<int, Tracks>& rhs);

  bool tracksOverlap(const Track& lhs, const Track& rhs);

  Track mergeTracks(const Tracks& tracks, int output_id);

  Tracks::const_iterator findTrackByID(const Tracks& tracks, int id);

  Tracks::iterator findTrackByID(Tracks& tracks, int id);

  void eraseTrack(int id);

  int current_id_ = 0;

  double angle_threshold_;
  double distance_threshold_;

  std::multiset<TrackIDWithSource, CompareId> tracks_sources_;

  Tracks tracks_;
};
}  // namespace track_merging
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_TRACK_UNIFYING_HPP
