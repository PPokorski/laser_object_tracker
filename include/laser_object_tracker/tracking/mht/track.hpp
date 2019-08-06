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

#ifndef LASER_OBJECT_TRACKER_TRACKING_MHT_TRACK_HPP
#define LASER_OBJECT_TRACKER_TRACKING_MHT_TRACK_HPP

#include <list>

#include <mht/list.h>

namespace laser_object_tracker {
namespace tracking {
namespace mht {
class TrackElement : public DLISTnode {
 public:
  TrackElement(double state_x,
               double state_y,
               double velocity_x,
               double velocity_y,
               double report_x,
               double report_y,
               double log_likelihood,
               int frame_number,
               size_t corner_id)
      : state_x_(state_x),
        state_y_(state_y),
        velocity_x_(velocity_x),
        velocity_y_(velocity_y),
        report_x_(report_x),
        report_y_(report_y),
        log_likelihood_(log_likelihood),
        frame_number_(frame_number),
        corner_id_(corner_id) {
    has_report_ = !(std::isnan(report_x) || std::isnan(report_y));
  }

  size_t getCornerId() const {
    return corner_id_;
  }

  double getStateX() const {
    return state_x_;
  }

  double getStateY() const {
    return state_y_;
  }

  double getVelocityX() const {
    return velocity_x_;
  }

  double getVelocityY() const {
    return velocity_y_;
  }

  double getReportX() const {
    return report_x_;
  }

  double getReportY() const {
    return report_y_;
  }

 protected:
  MEMBERS_FOR_DLISTnode(TrackElement)

 private:
  bool has_report_;

  double state_x_, state_y_,
         velocity_x_, velocity_y_,
         report_x_, report_y_;

  double log_likelihood_;

  int frame_number_;
  size_t corner_id_;
};

class Track : public DLISTnode {
 public:
  explicit Track(int id) : id_(id), track_() {}

  int getId() const {
    return id_;
  }

  std::list<TrackElement> track_;

 protected:
  MEMBERS_FOR_DLISTnode(Track)

 private:
  int id_;
};

}  // namespace mht
}  // namespace tracking
}  // namespace laser_object_tracker

#endif  // LASER_OBJECT_TRACKER_TRACKING_MHT_TRACK_HPP
