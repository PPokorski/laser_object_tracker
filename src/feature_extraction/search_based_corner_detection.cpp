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

#include "laser_object_tracker/feature_extraction/search_based_corner_detection.hpp"

#include <utility>

namespace laser_object_tracker {
namespace feature_extraction {

SearchBasedCornerDetection::SearchBasedCornerDetection(double theta_resolution,
                                                       CriterionFunctor criterion) :
    theta_resolution_(theta_resolution), criterion_(std::move(criterion)) {}

bool
SearchBasedCornerDetection::extractFeature(const data_types::LaserScanFragment& fragment, features::Feature& feature) {
  if (fragment.empty()) {
    throw std::invalid_argument("Passed fragment is empty.");
  }

  double best_assessment = -std::numeric_limits<double>::infinity();
  double best_angle = 0.0;
  Eigen::Vector2d projection_1, projection_2;
  Eigen::MatrixX2d points;
  Eigen::VectorXd projected_points_x, projected_points_y;

  fragmentToEigenMatrix(fragment, points);
  for (double theta = 0; theta < M_PI_2; theta += theta_resolution_) {
    projection_1 << std::cos(theta), std::sin(theta);
    projection_2 << -std::sin(theta), std::cos(theta);

    projected_points_x = points * projection_1;
    projected_points_y = points * projection_2;

    double assessment = criterion_(projected_points_x, projected_points_y);
    if (assessment > best_assessment) {
      best_assessment = assessment;
      best_angle = theta;
    }
  }

  projection_1 << std::cos(best_angle), std::sin(best_angle);
  projection_2 << -std::sin(best_angle), std::cos(best_angle);

  projected_points_x = points * projection_1;
  projected_points_y = points * projection_2;

  Eigen::Hyperplane<double, 2> edge_1, edge_2, edge_3, edge_4;
  edge_1.coeffs() << std::cos(best_angle), std::sin(best_angle), -projected_points_x.minCoeff();
  edge_2.coeffs() << -std::sin(best_angle), std::cos(best_angle), -projected_points_y.minCoeff();
  edge_3.coeffs() << std::cos(best_angle), std::sin(best_angle), -projected_points_x.maxCoeff();
  edge_4.coeffs() << -std::sin(best_angle), std::cos(best_angle), -projected_points_y.maxCoeff();

  feature.observation_ = findMatchingCorner(points.col(0), points.col(1),
                               edge_1,
                               edge_2,
                               edge_3,
                               edge_4);

  return true;
}

double SearchBasedCornerDetection::getThetaResolution() const {
  return theta_resolution_;
}

void SearchBasedCornerDetection::setThetaResolution(double theta_resolution) {
  theta_resolution_ = theta_resolution;
}

const SearchBasedCornerDetection::CriterionFunctor& SearchBasedCornerDetection::getCriterion() const {
  return criterion_;
}

void SearchBasedCornerDetection::setCriterion(const SearchBasedCornerDetection::CriterionFunctor& criterion) {
  criterion_ = criterion;
}

Eigen::VectorXd SearchBasedCornerDetection::findMatchingCorner(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                                                               const Eigen::Hyperplane<double, 2>& one,
                                                               const Eigen::Hyperplane<double, 2>& two,
                                                               const Eigen::Hyperplane<double, 2>& three,
                                                               const Eigen::Hyperplane<double, 2>& four) const {
  using Corner = std::pair<const Eigen::Hyperplane<double, 2> *,
                           const Eigen::Hyperplane<double, 2> *>;
  std::array<std::pair<Corner, double>, 4> corners_assessments{{
                                                                   {{&one, &two}, 0.0},
                                                                   {{&two, &three}, 0.0},
                                                                   {{&three, &four}, 0.0},
                                                                   {{&four, &one}, 0.0}
                                                               }};

  for (auto& assessment : corners_assessments) {
    assessment.second = assessCorner(x, y, *assessment.first.first, *assessment.first.second);
  }

  std::sort(corners_assessments.begin(), corners_assessments.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.second > rhs.second;
  });

  auto actual_corner = corners_assessments.begin();

  // We explicitly search for an opposite corner, because corner with the worst assessment may not be opposite to the
  // one with the best
  auto opposite_corner = std::find_if(actual_corner + 1, corners_assessments.end(),
                                      [&actual_corner, this](const auto& element) {
                                        return this->linesParallel(*actual_corner->first.first, *element.first.first) &&
                                            this->linesParallel(*actual_corner->first.second, *element.first.second);
                                      });

  if (opposite_corner == corners_assessments.end()) {
    throw std::logic_error("Was not able to found an opposite corner.");
  }

  Eigen::VectorXd corner(6);
  corner.head<2>() = actual_corner->first.first->intersection(
      *actual_corner->first.second);
  corner.segment<2>(2) = actual_corner->first.first->intersection(
      *opposite_corner->first.second);
  corner.tail<2>() = actual_corner->first.second->intersection(
      *opposite_corner->first.first);
  features::Corner2D corner_2_d;
  corner_2_d.corner_ = actual_corner->first.first->intersection(
      *actual_corner->first.second);
  corner_2_d.point_1_ = actual_corner->first.first->intersection(
      *opposite_corner->first.second);
  corner_2_d.point_2_ = actual_corner->first.second->intersection(
      *opposite_corner->first.first);

  return corner;
}

double SearchBasedCornerDetection::assessLine(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                                              const Eigen::Hyperplane<double, 2>& line) const {
  double assessment = 0.0;
  Eigen::Vector2d vector;
  for (int i = 0; i < x.size(); ++i) {
    vector << x(i), y(i);
    assessment += line.absDistance(vector);
  }

  return assessment;
}

double SearchBasedCornerDetection::assessCorner(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                                                const Eigen::Hyperplane<double, 2>& line_1,
                                                const Eigen::Hyperplane<double, 2>& line_2) const {
  double assessment = 0.0;
  Eigen::Vector2d xy;
  for (int i = 0; i < x.size(); ++i) {
    xy << x(i), y(i);
    assessment -= std::min(line_1.absDistance(xy),
                           line_2.absDistance(xy));
  }

  return assessment;
}

bool SearchBasedCornerDetection::linesParallel(const Eigen::Hyperplane<double, 2>& one,
                                               const Eigen::Hyperplane<double, 2>& two) const {
  return one.normal().isApprox(two.normal());
}

double areaCriterion(const Eigen::VectorXd& x, const Eigen::VectorXd& y) {
  if (x.size() == 0 || y.size() == 0) {
    return 0.0;
  }

  double x_min = x.minCoeff(), x_max = x.maxCoeff();
  double y_min = y.minCoeff(), y_max = y.maxCoeff();

  return -(x_max - x_min) * (y_max - y_min);
}

double closenessCriterion(const Eigen::VectorXd& x, const Eigen::VectorXd& y) {
  if (x.size() == 0 || y.size() == 0) {
    return 0.0;
  }

  double x_min = x.minCoeff(), x_max = x.maxCoeff();
  double y_min = y.minCoeff(), y_max = y.maxCoeff();

  Eigen::VectorXd lower_border_distance = x.array() - x_min,
      higher_border_distance = x_max - x.array();

  Eigen::VectorXd border_distance_x =
      (lower_border_distance.squaredNorm() < higher_border_distance.squaredNorm()) ?
      lower_border_distance : higher_border_distance;

  lower_border_distance = y.array() - y_min;
  higher_border_distance = y_max - y.array();

  Eigen::VectorXd border_distance_y =
      (lower_border_distance.squaredNorm() < higher_border_distance.squaredNorm()) ?
      lower_border_distance : higher_border_distance;

  double criterion_value = 0.0;
  for (int i = 0; i < border_distance_x.size(); ++i) {
    static constexpr double MIN_DISTANCE = 0.01;
    double distance = std::max(std::min(border_distance_x(i), border_distance_y(i)),
                               MIN_DISTANCE);

    criterion_value += 1 / distance;
  }

  return criterion_value;
}

double varianceCriterion(const Eigen::VectorXd& x, const Eigen::VectorXd& y) {
  if (x.size() == 0 || y.size() == 0) {
    return 0.0;
  }

  double x_min = x.minCoeff(), x_max = x.maxCoeff();
  double y_min = y.minCoeff(), y_max = y.maxCoeff();

  Eigen::VectorXd lower_border_distance = x.array() - x_min,
      higher_border_distance = x_max - x.array();

  Eigen::VectorXd border_distance_x =
      (lower_border_distance.squaredNorm() < higher_border_distance.squaredNorm()) ?
      lower_border_distance : higher_border_distance;

  lower_border_distance = y.array() - y_min;
  higher_border_distance = y_max - y.array();

  Eigen::VectorXd border_distance_y =
      (lower_border_distance.squaredNorm() < higher_border_distance.squaredNorm()) ?
      lower_border_distance : higher_border_distance;

  long classified_as_x_number = (border_distance_x.array() < border_distance_y.array()).count();
  long classified_as_y_number = (border_distance_y.array() < border_distance_x.array()).count();

  Eigen::VectorXd classified_as_x(classified_as_x_number),
      classified_as_y(classified_as_y_number);

  int i_x = 0;
  int i_y = 0;
  for (int i = 0; i < border_distance_x.size(); ++i) {
    if (border_distance_x(i) < border_distance_y(i)) {
      classified_as_x(i_x) = border_distance_x(i);
      ++i_x;
    } else if (border_distance_y(i) < border_distance_x(i)) {
      classified_as_y(i_y) = border_distance_y(i);
      ++i_y;
    }
  }

  double mean_x = i_x != 0 ? classified_as_x.mean() : 0.0;
  double mean_y = i_y != 0 ? classified_as_y.mean() : 0.0;
  double variance_x = i_x != 0 ? (classified_as_x.squaredNorm() / classified_as_x.size()) - mean_x * mean_x :
                      0.0;
  double variance_y = i_y != 0 ? (classified_as_y.squaredNorm() / classified_as_y.size()) - mean_y * mean_y :
                      0.0;

  return -variance_x - variance_y;
}
}  // namespace feature_extraction
}  // namespace laser_object_tracker
