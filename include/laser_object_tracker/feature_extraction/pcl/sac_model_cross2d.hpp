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

#ifndef LASER_OBJECT_TRACKER_SAMPLE_CONSENUS_MODEL_CORNER_2_D_HPP
#define LASER_OBJECT_TRACKER_SAMPLE_CONSENUS_MODEL_CORNER_2_D_HPP

#include <pcl/common/concatenate.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/eigen.h>
#include <pcl/sample_consensus/sac_model.h>

namespace pcl {

template <typename PointT>
class SampleConsenusModelCross2D : public SampleConsensusModel<PointT> {
 public:
    using SampleConsensusModel<PointT>::input_;
    using SampleConsensusModel<PointT>::indices_;
    using SampleConsensusModel<PointT>::error_sqr_dists_;

    using PointCloud = typename SampleConsensusModel<PointT>::PointCloud;
    using PointCloudPtr = typename SampleConsensusModel<PointT>::PointCloudPtr;
    using PointCloudConstPtr = typename SampleConsensusModel<PointT>::PointCloudConstPtr;

    using Ptr = boost::shared_ptr<SampleConsenusModelCross2D>;
    using CosntPtr = boost::shared_ptr<const SampleConsenusModelCross2D>;

    explicit SampleConsenusModelCross2D(const PointCloudConstPtr& cloud, bool random = false) :
            SampleConsensusModel<PointT>(cloud, random) {}

    SampleConsenusModelCross2D(const PointCloudConstPtr& cloud,
            const std::vector<int>& indices,
            bool random = false) :
            SampleConsensusModel<PointT>(cloud, indices, random) {}

    SampleConsenusModelCross2D(const SampleConsenusModelCross2D& other) :
            SampleConsensusModel<PointT>(SampleConsenusModelCross2D()) {}

    virtual ~SampleConsenusModelCross2D() = default;

    inline SampleConsenusModelCross2D& operator=(const SampleConsenusModelCross2D& other) {
        SampleConsensusModel<PointT>::operator=(other);
        return *this;
    }

    bool computeModelCoefficients(const std::vector<int>& samples, Eigen::VectorXf& model_coefficients) override {
        if (samples.size() != 3)
        {
            PCL_ERROR ("[pcl::SampleConsensusModelCross2D::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
            return false;
        }

        model_coefficients.resize(4);

        Eigen::Vector2d p0(input_->points[samples[0]].x, input_->points[samples[0]].y);
        Eigen::Vector2d p1(input_->points[samples[1]].x, input_->points[samples[1]].y);
        Eigen::Vector2d p2(input_->points[samples[2]].x, input_->points[samples[2]].y);

        Eigen::Vector2d d0 = p1 - p0;
        Eigen::Vector2d d1 = p2 - p1;
        Eigen::Vector2d d2 = p2 - p0;

        Eigen::Vector2d corner;
        Eigen::Vector2d point_1;
        Eigen::Vector2d point_2;

        if (vectorsPerpendicular(d0, d1))
        {
            corner = p1;
            orderCounterClockwise(p0, p2, p1, point_1, point_2);
        }
        else if (vectorsPerpendicular(d0, d2))
        {
            corner = p0;
            orderCounterClockwise(p1, p2, p0, point_1, point_2);
        }
        else
        {
            corner = p2;
            orderCounterClockwise(p0, p1, p2, point_1, point_2);
        }

        model_coefficients[0] = corner[0];
        model_coefficients[1] = corner[1];
        model_coefficients[2] = (point_1 - corner)(0);
        model_coefficients[3] = (point_1 - corner)(1);

        model_coefficients.tail<2>().normalize();
        return true;
    }

    void optimizeModelCoefficients(const std::vector<int>& inliers, const Eigen::VectorXf& model_coefficients,
                                   Eigen::VectorXf& optimized_coefficients) override {
        optimized_coefficients = model_coefficients;
        if (!isModelValid(model_coefficients))
        {
            PCL_ERROR ("[pcl::SampleConsensusModelCross2D::optimizeModelCoefficients] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size());
            return;
        }

        if (inliers.size() <= 3)
        {
            PCL_ERROR ("[pcl::SampleConsensusModelCross2D::optimizeModelCoefficients] Not enough inliers found to support a model (%lu)! Returning the same coefficients.\n", inliers.size());
            return;
        }

        tmp_inliers_ = &inliers;

        OptimizationFunctor functor(inliers.size(), this);
        Eigen::NumericalDiff<OptimizationFunctor> numerical_diff(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, float> levenberg_marquardt(numerical_diff);
        int info = levenberg_marquardt.minimize(optimized_coefficients);

        PCL_DEBUG ("[pcl::SampleConsensusModelCross2D::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g \nFinal solution: %g %g %g %g\n",
                   info, levenberg_marquardt.fvec.norm (), model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3], optimized_coefficients[0], optimized_coefficients[1], optimized_coefficients[2], optimized_coefficients[3]);
    }

    void getDistancesToModel(const Eigen::VectorXf& model_coefficients, std::vector<double>& distances) override {
        if (!isModelValid(model_coefficients))
        {
            distances.clear();
            PCL_ERROR ("[pcl::SampleConsensusModelCross2D::getDistancesToModel] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size());
            return;
        }

        distances.resize(indices_->size());

        Eigen::Vector4f corner_point(model_coefficients[0], model_coefficients[1], 0.0, 0.0);
        Eigen::Vector4f line_1_direction(model_coefficients[2], model_coefficients[3], 0.0, 0.0);
        Eigen::Vector4f line_2_direction(model_coefficients[3], -model_coefficients[2], 0.0, 0.0);
        line_1_direction.normalize();
        line_2_direction.normalize();

        Eigen::Vector4f point;
        for (size_t i = 0; i < indices_->size(); ++i)
        {
            pointToEigenVector(input_->points[(*indices_)[i]], point);
            distances[i] = std::sqrt(std::min(sqrPointToLineDistance(point, corner_point, line_1_direction),
                                              sqrPointToLineDistance(point, corner_point, line_2_direction)));
        }
    }

    void selectWithinDistance(const Eigen::VectorXf& model_coefficients, const double threshold,
                              std::vector<int>& inliers) override {
        if (!isModelValid(model_coefficients))
        {
            inliers.clear();
            PCL_ERROR ("[pcl::SampleConsensusModelCross2D::selectWithinDistance] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size());
            return;
        }

        double squared_threshold = threshold * threshold;
        inliers.resize(indices_->size());
        error_sqr_dists_.resize(indices_->size());

        Eigen::Vector4f corner_point(model_coefficients[0], model_coefficients[1], 0.0, 0.0);
        Eigen::Vector4f line_1_direction(model_coefficients[2], model_coefficients[3], 0.0, 0.0);
        Eigen::Vector4f line_2_direction(model_coefficients[3], -model_coefficients[2], 0.0, 0.0);
        line_1_direction.normalize();
        line_2_direction.normalize();

        Eigen::Vector4f point;
        int point_number = 0;
        for (size_t i = 0; i < indices_->size(); ++i)
        {
            pointToEigenVector(input_->points[(*indices_)[i]], point);
            double squared_distance = std::min(sqrPointToLineDistance(point, corner_point, line_1_direction),
                                               sqrPointToLineDistance(point, corner_point, line_2_direction));

            if (squared_distance < squared_threshold)
            {
                inliers[point_number] = (*indices_)[i];
                error_sqr_dists_[point_number] = squared_distance;
                ++point_number;
            }
        }
        inliers.resize(point_number);
        error_sqr_dists_.resize(point_number);
    }

    int countWithinDistance(const Eigen::VectorXf& model_coefficients, const double threshold) override {
        if (!isModelValid(model_coefficients))
        {
            PCL_ERROR ("[pcl::SampleConsensusModelCross2D::countWithinDistance] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size());
            return 0;
        }

        double squared_threshold = threshold * threshold;

        Eigen::Vector4f corner_point(model_coefficients[0], model_coefficients[1], 0.0, 0.0);
        Eigen::Vector4f line_1_direction(model_coefficients[2], model_coefficients[3], 0.0, 0.0);
        Eigen::Vector4f line_2_direction(model_coefficients[3], -model_coefficients[2], 0.0, 0.0);
        line_1_direction.normalize();
        line_2_direction.normalize();

        Eigen::Vector4f point;
        int point_number = 0;
        for (size_t i = 0; i < indices_->size(); ++i)
        {
            pointToEigenVector(input_->points[(*indices_)[i]], point);
            double squared_distance = std::min(sqrPointToLineDistance(point, corner_point, line_1_direction),
                                               sqrPointToLineDistance(point, corner_point, line_2_direction));

            if (squared_distance < squared_threshold)
            {
                ++point_number;
            }
        }

        return point_number;
    }

    void projectPoints(const std::vector<int>& inliers, const Eigen::VectorXf& model_coefficients,
                       PointCloud& projected_points, bool copy_data_fields = true) override {
        if (!isModelValid(model_coefficients))
        {
            PCL_ERROR ("[pcl::SampleConsensusModelCross2D::projectPoints] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size());
            return;
        }

        Eigen::Vector4f corner_point(model_coefficients[0], model_coefficients[1], 0.0, 0.0);
        Eigen::Vector4f line_1_direction(model_coefficients[2], model_coefficients[3], 0.0, 0.0);
        Eigen::Vector4f line_2_direction(model_coefficients[3], -model_coefficients[2], 0.0, 0.0);
        line_1_direction.normalize();
        line_2_direction.normalize();

        projected_points.header = input_->header;
        projected_points.is_dense = input_->is_dense;

        if (copy_data_fields)
        {
            projected_points.points.resize(input_->points.size());
            projected_points.width = input_->width;
            projected_points.height = input_->height;

            using FieldList = typename pcl::traits::fieldList<PointT>::type;
            for (size_t i = 0; i < projected_points.points.size(); ++i)
            {
                pcl::for_each_type<FieldList>(NdConcatenateFunctor<PointT, PointT>(input_->points[i], projected_points[i]));
            }

            Eigen::Vector4f point;
            Eigen::Vector4f projected_point;
            Eigen::Vector4f line_direction;
            for (size_t i = 0; i < inliers.size(); ++i)
            {
                pointToEigenVector(input_->points[inliers[i]], point);
                if (sqrPointToLineDistance(point, corner_point, line_1_direction) <
                    sqrPointToLineDistance(point, corner_point, line_2_direction))
                {
                    line_direction = line_1_direction;
                }
                else
                {
                    line_direction = line_2_direction;
                }

                float k = (point.dot(line_direction) - corner_point.dot(line_direction)) /
                        line_direction.dot(line_direction);

                projected_point = corner_point + k * line_direction;
                projected_points.points[inliers[i]].x = projected_point[0];
                projected_points.points[inliers[i]].y = projected_point[1];
                projected_points.points[inliers[i]].z = projected_point[2];
            }
        }
        else
        {
            projected_points.points.resize(inliers.size());
            projected_points.width = static_cast<uint32_t>(inliers.size());
            projected_points.height = 1;

            using FieldList = typename pcl::traits::fieldList<PointT>::type;
            for (size_t i = 0; i < projected_points.points.size(); ++i)
            {
                pcl::for_each_type<FieldList>(NdConcatenateFunctor<PointT, PointT>(input_->points[inliers[i]], projected_points[i]));
            }

            Eigen::Vector4f point;
            Eigen::Vector4f projected_point;
            Eigen::Vector4f line_direction;
            for (size_t i = 0; i < inliers.size(); ++i)
            {
                pointToEigenVector(input_->points[inliers[i]], point);
                if (sqrPointToLineDistance(point, corner_point, line_1_direction) <
                    sqrPointToLineDistance(point, corner_point, line_2_direction))
                {
                    line_direction = line_1_direction;
                }
                else
                {
                    line_direction = line_2_direction;
                }

                float k = (point.dot(line_direction) - corner_point.dot(line_direction)) /
                          line_direction.dot(line_direction);

                projected_point = corner_point + k * line_direction;
                projected_points.points[i].x = projected_point[0];
                projected_points.points[i].y = projected_point[1];
                projected_points.points[i].z = projected_point[2];
            }
        }
    }

    bool doSamplesVerifyModel(const std::set<int>& indices, const Eigen::VectorXf& model_coefficients,
                              const double threshold) override {
        if (!isModelValid(model_coefficients))
        {
            PCL_ERROR ("[pcl::SampleConsensusModelCross2D::doSamplesVerifyModel] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size());
            return false;
        }

        Eigen::Vector4f corner_point(model_coefficients[0], model_coefficients[1], 0.0, 0.0);
        Eigen::Vector4f line_1_direction(model_coefficients[2], model_coefficients[3], 0.0, 0.0);
        Eigen::Vector4f line_2_direction(model_coefficients[3], -model_coefficients[2], 0.0, 0.0);
        line_1_direction.normalize();
        line_2_direction.normalize();

        double squared_threshold = threshold * threshold;

        Eigen::Vector4f point;
        for (int i : indices)
        {
            pointToEigenVector(input_->points[i], point);
            double squared_distance = std::min(sqrPointToLineDistance(point, corner_point, line_1_direction),
                                               sqrPointToLineDistance(point, corner_point, line_2_direction));

            if (squared_distance > squared_threshold)
            {
                return false;
            }
        }
        return true;
    }

    SacModel getModelType() const override {
        return SACMODEL_PARALLEL_PLANE;
    }

 protected:
    bool isModelValid(const Eigen::VectorXf& model_coefficients) override {
        if (model_coefficients.size() != 4)
        {
            PCL_ERROR("[pcl::SampleConsensusModelCross2D::isModelValid] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size());
            return false;
        }

        if (model_coefficients(2) == 0.0 &&
            model_coefficients(3) == 0.0)
        {
            PCL_ERROR("[pcl::SampleConsensusModelCross2D::isModelValid] Direction of line can't be 0.0, 0.0!\n");
        }

        return true;
    }

    bool isSampleGood(const std::vector<int>& samples) const override {
        Eigen::Vector2d p0(input_->points[samples[0]].x, input_->points[samples[0]].y);
        Eigen::Vector2d p1(input_->points[samples[1]].x, input_->points[samples[1]].y);
        Eigen::Vector2d p2(input_->points[samples[2]].x, input_->points[samples[2]].y);

        if (p0.isApprox(p1) &&
            p0.isApprox(p2))
        {
            return false;
        }

        Eigen::Vector2d d0 = p1 - p0;
        Eigen::Vector2d d1 = p2 - p1;
        Eigen::Vector2d d2 = p2 - p0;

        return vectorsPerpendicular(d0, d1) ||
               vectorsPerpendicular(d0, d2) ||
               vectorsPerpendicular(d1, d2);
    }

 private:
    struct OptimizationFunctor : pcl::Functor<float> {
        OptimizationFunctor(int m_data_points, SampleConsenusModelCross2D<PointT> *model) :
                Functor(m_data_points),
                model_(model) {}

        int operator() (const Eigen::VectorXf& x, Eigen::VectorXf& fvec) const {
            Eigen::Vector4f corner_point(x[0], x[1], 0.0, 0.0);
            Eigen::Vector4f line_1_direction(x[2], x[3], 0.0, 0.0);
            Eigen::Vector4f line_2_direction(x[3], -x[2], 0.0, 0.0);
            line_1_direction.normalize();
            line_2_direction.normalize();

            Eigen::Vector4f point;
            for (int i = 0; i < values(); ++i)
            {
                model_->pointToEigenVector(model_->input_->points[i], point);
                double squared_distance = std::min(sqrPointToLineDistance(point, corner_point, line_1_direction),
                                                   sqrPointToLineDistance(point, corner_point, line_2_direction));

                fvec[i] = squared_distance;
            }

            return 0;
        }

        pcl::SampleConsenusModelCross2D<PointT>* model_;
    };

    bool vectorsPerpendicular(const Eigen::Vector2d& one, const Eigen::Vector2d& two) const {
        static constexpr float ZERO_TOLERANCE = 0.0001f;
        return std::fabs(one.dot(two)) < ZERO_TOLERANCE;
    }

    void orderCounterClockwise(const Eigen::Vector2d& one, const Eigen::Vector2d& two,
                               const Eigen::Vector2d& relative,
                               Eigen::Vector2d& first_out, Eigen::Vector2d& second_out) const {
        Eigen::Vector2d first_vec = one - relative;
        Eigen::Vector2d second_vec = two - relative;
        if (std::atan2(first_vec[1], first_vec[0]) < std::atan2(second_vec[1], second_vec[0]))
        {
            first_out = one;
            second_out = two;
        }
        else
        {
            first_out = two;
            second_out = one;
        }
    }

    void pointToEigenVector(const PointT& point, Eigen::Vector4f& vector) {
        vector << point.x, point.y, 0.0, 0.0;
    }

    const std::vector<int> *tmp_inliers_;
};

}  // namespace pcl

#endif //LASER_OBJECT_TRACKER_SAMPLE_CONSENUS_MODEL_CORNER_2_D_HPP
