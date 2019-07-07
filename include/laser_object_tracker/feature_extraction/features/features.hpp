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

#ifndef LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_FEATURES_HPP
#define LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_FEATURES_HPP

#include <vector>

#include <Eigen/Core>

namespace laser_object_tracker {
namespace feature_extraction {
namespace features {

struct Feature {
  Eigen::VectorXd observation_;

  std::vector<int> vector_int_;
  std::vector<bool> vector_bool_;
};

struct Point2D {
  Point2D() = default;

  explicit Point2D(const Eigen::VectorXd& coefficients) {
    if (coefficients.size() != 2) {
      throw std::invalid_argument("Point2D needs 2 coefficients. " +
          std::to_string(coefficients.size()) + " provided.");
    }
    point_ = coefficients.head<2>();
  }

  Eigen::Vector2d point_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Corner2D {
  Corner2D() = default;

  explicit Corner2D(const Eigen::VectorXd& coefficients) {
    if (coefficients.size() != 6) {
      throw std::invalid_argument("Corner2D needs 6 coefficients. " +
          std::to_string(coefficients.size()) + " provided.");
    }

    corner_ = coefficients.head<2>();
    point_1_ = coefficients.segment<2>(2);
    point_2_ = coefficients.tail<2>();
  }
  Eigen::Vector2d corner_, point_1_, point_2_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

using Corners2D = std::vector<Corner2D, Eigen::aligned_allocator<Corner2D>>;

struct Segment2D {
  Segment2D() = default;

  Segment2D(const Eigen::Vector2d& start, const Eigen::Vector2d& end) :
      start_(start),
      end_(end) {}
  explicit Segment2D(const Eigen::VectorXd& coefficients) {
    if (coefficients.size() != 4) {
      throw std::invalid_argument("Segment2D needs 4 coefficients. " +
          std::to_string(coefficients.size()) + " provided.");
    }

    start_ = coefficients.head<2>();
    end_ = coefficients.tail<2>();
  }
  Eigen::Vector2d start_, end_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

using Segments2D = std::vector<Segment2D, Eigen::aligned_allocator<Segment2D>>;

struct MultiSegment2D {
  MultiSegment2D() = default;

  explicit MultiSegment2D(const Eigen::VectorXd& coefficients) {
    if (coefficients.size() % 4 != 0) {
      throw std::invalid_argument("MultiSegment2D needs multiple of 4 coefficients. " +
          std::to_string(coefficients.size()) + " provided.");
    }

    segments_.resize(coefficients.size() / 4);
    for (int i = 0; i < segments_.size(); ++i) {
      segments_.at(i) = Segment2D(coefficients.segment<4>(4 * i));
    }
  }

  Segments2D segments_;
};

using MultiSegments2D = std::vector<MultiSegment2D>;
}  // namespace features
}  // namespace feature_extraction
}  // namespace laser_object_tracker

#endif  // LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_FEATURES_HPP
