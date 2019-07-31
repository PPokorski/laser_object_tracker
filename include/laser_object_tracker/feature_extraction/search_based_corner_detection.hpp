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

#ifndef LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_SEARCH_BASED_CORNER_DETECTION_HPP
#define LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_SEARCH_BASED_CORNER_DETECTION_HPP

#include "laser_object_tracker/feature_extraction/base_feature_extraction.hpp"
#include "laser_object_tracker/feature_extraction/features/object.hpp"

namespace laser_object_tracker {
namespace feature_extraction {

class SearchBasedCornerDetection : public BaseFeatureExtraction<features::Feature> {
 public:
  using CriterionFunctor = std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>;

  SearchBasedCornerDetection(double theta_resolution, CriterionFunctor criterion);

  bool extractFeature(const data_types::LaserScanFragment& fragment, FeatureT& feature) override;

  double getThetaResolution() const;

  void setThetaResolution(double theta_resolution);

  const CriterionFunctor& getCriterion() const;

  void setCriterion(const CriterionFunctor& criterion);

 private:
  Eigen::VectorXd findMatchingCorner(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                                     const Eigen::Hyperplane<double, 2>& one,
                                     const Eigen::Hyperplane<double, 2>& two,
                                     const Eigen::Hyperplane<double, 2>& three,
                                     const Eigen::Hyperplane<double, 2>& four) const;

  double assessLine(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                    const Eigen::Hyperplane<double, 2>& line) const;

  double assessCorner(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                      const Eigen::Hyperplane<double, 2>& line_1, const Eigen::Hyperplane<double, 2>& line_2) const;

  bool linesParallel(const Eigen::Hyperplane<double, 2>& one,
                     const Eigen::Hyperplane<double, 2>& two) const;

  double theta_resolution_;
  CriterionFunctor criterion_;
};

double areaCriterion(const Eigen::VectorXd& x, const Eigen::VectorXd& y);
double closenessCriterion(const Eigen::VectorXd& x, const Eigen::VectorXd& y);
double varianceCriterion(const Eigen::VectorXd& x, const Eigen::VectorXd& y);
}  // namespace feature_extraction
}  // namespace laser_object_tracker

#endif  // LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_SEARCH_BASED_CORNER_DETECTION_HPP
