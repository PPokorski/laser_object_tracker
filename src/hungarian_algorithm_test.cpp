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

#include <chrono>
#include <iostream>
#include <numeric>
#include <iomanip>

#include "laser_object_tracker/data_association/hungarian_algorithm.hpp"

using namespace std;
using namespace std::chrono;

static constexpr int MAX_SIZE = 50;
static constexpr int ITERATIONS_PER_SIZE = 100000;

int main() {
  laser_object_tracker::data_association::HungarianAlgorithm hungarian_algorithm;
  Eigen::VectorXi assignment;

  int width = 20;
  cout << setw(width) << "Size" << ","
       << setw(width) << "Iterations" << ","
       << setw(width) << "Cumulative time" << ","
       << setw(width) << "Mean time" << ","
       << setw(width) << "Min time" << ","
       << setw(width) << "Max time" << std::endl;

  for (int size = 1; size <= MAX_SIZE; ++size) {
    double cumulative_time = 0.0;
    double max_time = -numeric_limits<double>::infinity();
    double min_time = +numeric_limits<double>::infinity();
    high_resolution_clock::time_point begin, end;
    for (int i = 0; i < ITERATIONS_PER_SIZE; ++i) {
      begin = high_resolution_clock::now();
      hungarian_algorithm.solve(Eigen::MatrixXd::Random(size, size), hungarian_algorithm.NOT_NEEDED, assignment);
      end = high_resolution_clock::now();

      std::chrono::duration<double> duration = end - begin;
      double time_passed = duration.count();

      cumulative_time += time_passed;
      max_time = std::max(max_time, time_passed);
      min_time = std::min(min_time, time_passed);
    }

    cout << setw(width) << size << ","
         << setw(width) << ITERATIONS_PER_SIZE << ","
         << setw(width) << cumulative_time << ","
         << setw(width) << cumulative_time / ITERATIONS_PER_SIZE << ","
         << setw(width) << min_time << ","
         << setw(width) << max_time << std::endl;
  }

  return 0;
}