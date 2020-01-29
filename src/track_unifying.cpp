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

#include "laser_object_tracker/track_unifying.hpp"

namespace laser_object_tracker {
namespace track_unifying {
TrackUnifying::TrackUnifying(double angle_threshold, double distance_threshold)
    : angle_threshold_(angle_threshold), distance_threshold_(distance_threshold) {}

std::vector<tracking::ObjectTrack> TrackUnifying::unifyTracks(const std::map<int, Tracks>& tracks) {
  for (const auto& id_tracks : tracks) {
    for (const auto& track : id_tracks.second) {
      if (tracks_sources_.count(TrackSource{id_tracks.first, track.id_}) == 0) {
        tracks_sources_.insert({current_id_++, {id_tracks.first, track.id_}});
      }
    }
  }

  for (auto it_1 = tracks.begin(); it_1 != tracks.end(); ++it_1) {
    for (auto it_2 = std::next(it_1); it_2 != tracks.end(); ++it_2) {
      unifySourcePair(*it_1, *it_2);
    }
  }

  tracks_.reserve(tracks_sources_.size());
  for (auto it = tracks_sources_.begin(); it != tracks_sources_.end();) {
    int id = it->id;
    auto it_end = tracks_sources_.upper_bound(*it);
    Tracks filtered_tracks;
    for (; it != it_end; ++it) {
      auto& track = tracks.at(it->source.tracker_id);
      auto track_it = findTrackByID(track, it->source.track_id);
      if (track_it != track.end()) {
        filtered_tracks.push_back(*track_it);
      } else {
        auto it_to_erase = it--; // Set it to previous position
        if (tracks_sources_.count(it_to_erase->id) == 1) {
          eraseTrack(it_to_erase->id);
        }
        tracks_sources_.erase(it_to_erase);
      }
    }

    if (!filtered_tracks.empty()) {
      auto track_element = mergeTracks(filtered_tracks, id).track_.back();
      auto track_it = findTrackByID(tracks_, id);

      if (track_it != tracks_.end()) {
        track_it->track_.push_back(track_element);
      } else {
        tracks_.push_back({id, {track_element}});
      }
    }
  }
  return tracks_;
}

void TrackUnifying::unifySourcePair(const std::pair<int, Tracks>& lhs,
                                    const std::pair<int, Tracks>& rhs) {
  for (auto it_1 = lhs.second.begin(); it_1 != lhs.second.end(); ++it_1) {
    for (auto it_2 = rhs.second.begin(); it_2 != rhs.second.end(); ++it_2) {
      if (tracksOverlap(*it_1, *it_2)) {
        TrackSource source_1 {lhs.first, it_1->id_};
        TrackSource source_2 {rhs.first, it_2->id_};

        auto set_it_1 = tracks_sources_.find(source_1);
        auto set_it_2 = tracks_sources_.find(source_2);
        if (set_it_1 == tracks_sources_.end() || set_it_2 == tracks_sources_.end()) {
          continue;
        }
        if (set_it_1->id == set_it_2->id) {
          continue;
        }

        auto first_id = set_it_1->id <= set_it_2->id ? set_it_1 : set_it_2;
        auto second_id = set_it_1->id <= set_it_2->id ? set_it_2 : set_it_1;

        int id = first_id->id;
        TrackSource source = second_id->source;

        tracks_sources_.erase(second_id);
        tracks_sources_.insert({id, source});
      }
    }
  }
}

bool TrackUnifying::tracksOverlap(const TrackUnifying::Track& lhs, const TrackUnifying::Track& rhs) {
  if (lhs.track_.empty() || rhs.track_.empty()) {
    return false;
  }

  // Take polyline of the last TrackElement
  auto& last_polyline_lhs = lhs.track_.back().polyline_;
  auto& last_polyline_rhs = rhs.track_.back().polyline_;

  if (last_polyline_lhs.size() < 2 || last_polyline_rhs.size() < 2) {
    return false;
  }
  for (auto it_1 = last_polyline_lhs.begin(); it_1 != last_polyline_lhs.end(); ++it_1) {
    for (auto it_2 = last_polyline_rhs.begin(); it_2 != last_polyline_rhs.end(); ++it_2) {
      using Segment = feature_extraction::features::Segment2D;

      if ((it_1 == std::prev(last_polyline_lhs.end()) && last_polyline_lhs.size() == 2) ||
          (it_2 == std::prev(last_polyline_rhs.end()) && last_polyline_rhs.size() == 2)) {
        continue;
      }
      auto next_it1 = std::next(it_1) == last_polyline_lhs.end() ? last_polyline_lhs.begin() : std::next(it_1);
      auto next_it2 = std::next(it_2) == last_polyline_rhs.end() ? last_polyline_rhs.begin() : std::next(it_2);

      Segment segment_lhs(*it_1, *next_it1), segment_rhs(*it_2, *next_it2);
      if (feature_extraction::features::angleBetweenSegments(segment_lhs, segment_rhs) < angle_threshold_ &&
          feature_extraction::features::distanceBetweenSegments(segment_lhs, segment_rhs) < distance_threshold_) {
        return true;
      }
    }
  }
  return false;
}

TrackUnifying::Track TrackUnifying::mergeTracks(const Tracks& tracks, int output_id) {
  std::vector<tracking::ObjectTrackElement> last_positions;
  last_positions.reserve(tracks.size());
  for (const auto& track : tracks) {
    if (tracks.size() < 2 || track.track_.back().was_updated_) {
      last_positions.push_back(track.track_.back());
    }
  }

  tracking::ObjectTrackElement output_position {};
  for (const auto& position : last_positions) {
    output_position.position_ += position.position_;
    output_position.velocity_ += position.velocity_;
  }

  output_position.position_ /= last_positions.size();
  output_position.velocity_ /= last_positions.size();

  Track output_track;
  output_track.id_ = output_id;
  output_track.track_.push_back(output_position);

  return output_track;
}

TrackUnifying::Tracks::const_iterator TrackUnifying::findTrackByID(const Tracks& tracks, int id) {
  return std::find_if(tracks.begin(), tracks.end(), [id](const Track& track) {
    return track.id_ == id;
  });
}

TrackUnifying::Tracks::iterator TrackUnifying::findTrackByID(Tracks& tracks, int id) {
  return std::find_if(tracks.begin(), tracks.end(), [id](const Track& track) {
    return track.id_ == id;
  });
}

void TrackUnifying::eraseTrack(int id) {
  tracks_.erase(std::remove_if(tracks_.begin(),
                               tracks_.end(),
                               [id](const Track& track) {
                                 return track.id_ == id;}),
                tracks_.end());

}
}  // namespace track_merging
}  // namespace laser_object_tracker
