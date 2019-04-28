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

#include "laser_object_tracker/data_association/hungarian_algorithm.hpp"

namespace laser_object_tracker {
namespace data_association {

HungarianAlgorithm::HungarianAlgorithm(double max_allowed_cost) : BaseDataAssociation(max_allowed_cost) {}

double HungarianAlgorithm::solve(const Eigen::MatrixXd& cost_matrix,
                                 const Eigen::MatrixXd& covariance_matrix,
                                 Eigen::VectorXi& assignment_vector) {
  if (cost_matrix.size() == 0) {
    assignment_vector.setConstant(cost_matrix.cols(), NO_ASSIGNMENT);
    return 0.0;
  }

  min_dimension_ = std::min(cost_matrix.rows(), cost_matrix.cols());
  Eigen::ArrayXXd cost_matrix_copy = cost_matrix.array();
  assignment_vector.setConstant(cost_matrix.cols(), NO_ASSIGNMENT);

  star_matrix_.setConstant(cost_matrix.rows(), cost_matrix.cols(), false);
  new_star_matrix_.setConstant(cost_matrix.rows(), cost_matrix.cols(), false);
  prime_matrix_.setConstant(cost_matrix.rows(), cost_matrix.cols(), false);
  covered_columns_.setConstant(cost_matrix.cols(), false);
  covered_rows_.setConstant(cost_matrix.rows(), false);


  // Call solving function
  return assignmentOptimal(cost_matrix, cost_matrix_copy, assignment_vector);
}

double HungarianAlgorithm::assignmentOptimal(const Eigen::MatrixXd& cost_matrix,
                                             Eigen::ArrayXXd& cost_matrix_copy,
                                             Eigen::VectorXi& assignment) {
  // Preliminary steps
  if (cost_matrix_copy.rows() <= cost_matrix_copy.cols()) {
    // Find the smallest element in the row and subtract it from each element of the row
    cost_matrix_copy.colwise() -= cost_matrix_copy.rowwise().minCoeff();

    // Steps 1 and 2a
    for (int row = 0; row < cost_matrix_copy.rows(); ++row) {
      for (int col = 0; col < cost_matrix_copy.cols(); ++col) {
        if (isZero(cost_matrix_copy(row, col)) &&
            !covered_columns_(col)) {
          star_matrix_(row, col) = true;
          covered_columns_(col) = true;
          break;
        }
      }
    }
  } else {
    // Find the smallest element in the column and subtract it from each element of the column
    cost_matrix_copy.rowwise() -= cost_matrix_copy.colwise().minCoeff();

    // Steps 1 and 2a
    for (int col = 0; col < cost_matrix_copy.cols(); ++col) {
      for (int row = 0; row < cost_matrix_copy.rows(); ++row) {
        if (isZero(cost_matrix_copy(row, col)) &&
            !covered_rows_(row)) {
          star_matrix_(row, col) = true;
          covered_columns_(col) = true;
          covered_rows_(row) = true;
          break;
        }
      }
    }

    covered_rows_.setConstant(false);
  }

  // Move to step 2b
  step2b(cost_matrix, cost_matrix_copy, assignment);

  // Compute assignment cost
  return computeAssignmentCost(cost_matrix, cost_matrix_copy, assignment);
}

void HungarianAlgorithm::buildAssignmentVector(const Eigen::MatrixXd& cost_matrix,
                                               Eigen::ArrayXXd& cost_matrix_copy,
                                               Eigen::VectorXi& assignment) {
  for (int row = 0; row < star_matrix_.rows(); ++row) {
    for (int col = 0; col < star_matrix_.cols(); ++col) {
      if (star_matrix_(row, col) &&
          cost_matrix(row, col) <= max_allowed_cost_) {
        assignment(col) = row;
        break;
      }
    }
  }
}

double HungarianAlgorithm::computeAssignmentCost(const Eigen::MatrixXd& cost_matrix,
                                                 Eigen::ArrayXXd& cost_matrix_copy,
                                                 Eigen::VectorXi& assignment) {
  double cost = 0.0;
  int row = 0;

  for (int col = 0; col < cost_matrix.cols(); ++col) {
    row = assignment(col);
    if (row != NO_ASSIGNMENT) {
      cost += cost_matrix(row, col);
    }
  }
  return cost;
}

void HungarianAlgorithm::step2a(const Eigen::MatrixXd& cost_matrix,
                                Eigen::ArrayXXd& cost_matrix_copy,
                                Eigen::VectorXi& assignment) {
  // Cover every column containing a starred zero
  covered_columns_ = covered_columns_ || star_matrix_.colwise().any();

  step2b(cost_matrix, cost_matrix_copy, assignment);
}

void HungarianAlgorithm::step2b(const Eigen::MatrixXd& cost_matrix,
                                Eigen::ArrayXXd& cost_matrix_copy,
                                Eigen::VectorXi& assignment) {
  // Count covered columns
  if (covered_columns_.count() == min_dimension_) {
    // Algorithm finished
    buildAssignmentVector(cost_matrix, cost_matrix_copy, assignment);
  } else {
    // Move to step 3
    step3(cost_matrix, cost_matrix_copy, assignment);
  }
}

void HungarianAlgorithm::step3(const Eigen::MatrixXd& cost_matrix,
                               Eigen::ArrayXXd& cost_matrix_copy,
                               Eigen::VectorXi& assignment) {
  bool zeros_found = true;
  while (zeros_found) {
    zeros_found = false;
    for (int col = 0; col < cost_matrix_copy.cols(); ++col) {
      if (!covered_columns_(col)) {
        for (int row = 0; row < cost_matrix_copy.rows(); ++row) {
          if (!covered_rows_(row) &&
              isZero(cost_matrix_copy(row, col))) {
            // Prime zero
            prime_matrix_(row, col) = true;

            Eigen::Index starred_column;
            // If not found starred zero in current row
            if (!star_matrix_.row(row).maxCoeff(&starred_column)) {
              // Move to step 4
              step4(cost_matrix, cost_matrix_copy, assignment, row, col);
              return;
            } else {
              covered_rows_(row) = true;
              covered_columns_(starred_column) = false;
              zeros_found = true;
              break;
            }
          }
        }
      }
    }
  }

  // Move to step 5
  step5(cost_matrix, cost_matrix_copy, assignment);
}

void HungarianAlgorithm::step4(const Eigen::MatrixXd& cost_matrix,
                               Eigen::ArrayXXd& cost_matrix_copy,
                               Eigen::VectorXi& assignment,
                               int row,
                               int column) {
  // Generate temporary copy of star_matrix_
  new_star_matrix_ = star_matrix_;
  // Star current zero
  new_star_matrix_(row, column) = true;

  Eigen::Index star_row, star_column = column;
  // Find starred zero in current column
  bool star_found = star_matrix_.col(star_column).maxCoeff(&star_row);

  while (star_found) {
    // Unstar the starred zero
    new_star_matrix_(star_row, star_column) = false;

    Eigen::Index prime_row = star_row, prime_col;
    // Find primed zero in current row
    prime_matrix_.row(prime_row).maxCoeff(&prime_col);

    // Star the primed zero
    new_star_matrix_(prime_row, prime_col) = true;

    star_column = prime_col;
    // Find starred zero in current column
    star_found = star_matrix_.col(star_column).maxCoeff(&star_row);
  }

  // Use temporary copy as new star_matrix_
  // delete all primes, uncover all rows
  prime_matrix_.setConstant(false);
  star_matrix_ = new_star_matrix_;
  covered_rows_.setConstant(false);

  // Move to step 2a
  step2a(cost_matrix, cost_matrix_copy, assignment);
}

void HungarianAlgorithm::step5(const Eigen::MatrixXd& cost_matrix,
                               Eigen::ArrayXXd& cost_matrix_copy,
                               Eigen::VectorXi& assignment) {
  // We need to find the smallest uncovered element.
  // In order to do this we create matrix indicating which elements are not covered.
  // The element is not covered, if its row and column is not covered, hence multiplication representing logical AND.
  double min_uncovered = ((!covered_rows_).matrix() * (!covered_columns_).matrix())
      // If the element is not covered, take its value from cost_matrix, otherwise its max
      .select(cost_matrix_copy, std::numeric_limits<double>::max())
          // Find the smallest element
      .minCoeff();

  // Add min_uncovered to each covered row
  cost_matrix_copy = covered_rows_.replicate(1, covered_columns_.size()).
      select(cost_matrix_copy + min_uncovered, cost_matrix_copy);
  // Subtract min_uncovered from each uncovered column
  cost_matrix_copy = covered_columns_.replicate(covered_rows_.size(), 1).
      select(cost_matrix_copy, cost_matrix_copy - min_uncovered);

  // Move to step 3
  step3(cost_matrix, cost_matrix_copy, assignment);
}

bool HungarianAlgorithm::isZero(double number, double precision) {
  return std::abs(number) <= precision;
}
}  // namespace data_association
}  // namespace laser_object_tracker
