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

#include "laser_object_tracker/data_types/laser_scan_fragment.hpp"

// ROS
#include <pcl_conversions/pcl_conversions.h>

namespace laser_object_tracker {
namespace data_types {

LaserScanFragment LaserScanFragment::LaserScanFragmentFactory::fromLaserScan(const LaserScanType& laser_scan) {
  LaserScanFragment fragment;
  fragment.laser_scan_ = laser_scan;

  completeInitialization(fragment);

  return fragment;
}

LaserScanFragment LaserScanFragment::LaserScanFragmentFactory::fromLaserScan(LaserScanType&& laser_scan) {
  LaserScanFragment fragment;
  fragment.laser_scan_ = std::move(laser_scan);

  completeInitialization(fragment);

  return fragment;
}

void LaserScanFragment::LaserScanFragmentFactory::completeInitialization(LaserScanFragment& fragment) {
  if (fragment.laser_scan_.ranges.empty()) {
    return;
  }

  sensor_msgs::PointCloud2 pcl2;
  laser_projector_.projectLaser(fragment.laser_scan_, pcl2);
  pcl::moveFromROSMsg(pcl2, fragment.laser_scan_cloud_);

  fragment.occlusion_vector_.resize(fragment.laser_scan_.ranges.size(), false);

  auto range_it = fragment.laser_scan_.ranges.cbegin();
  auto pcl_it = fragment.laser_scan_cloud_.begin();
  for (; range_it != fragment.laser_scan_.ranges.cend(); ++range_it, ++pcl_it) {
    if (*range_it < fragment.getRangeMin() || *range_it >= fragment.getRangeMax()) {
      pcl_it = fragment.laser_scan_cloud_.insert(pcl_it, {std::numeric_limits<float>::quiet_NaN(),
                                                          std::numeric_limits<float>::quiet_NaN(),
                                                          std::numeric_limits<float>::quiet_NaN()});
    }
  }

  fragment.initializeInternalContainer();
}

LaserScanFragment::LaserScanFragment(const LaserScanFragment& other) noexcept :
    laser_scan_(other.laser_scan_),
    occlusion_vector_(other.occlusion_vector_),
    laser_scan_cloud_(other.laser_scan_cloud_) {
  initializeInternalContainer();
}

LaserScanFragment::LaserScanFragment(LaserScanFragment&& other) noexcept :
    laser_scan_(std::move(other.laser_scan_)),
    occlusion_vector_(std::move(other.occlusion_vector_)),
    laser_scan_cloud_(std::move(other.laser_scan_cloud_)) {
  initializeInternalContainer();
}

LaserScanFragment& LaserScanFragment::operator=(const LaserScanFragment& other) noexcept {
  laser_scan_ = other.laser_scan_;
  occlusion_vector_ = other.occlusion_vector_;
  laser_scan_cloud_ = other.laser_scan_cloud_;

  initializeInternalContainer();

  return *this;
}

LaserScanFragment& LaserScanFragment::operator=(LaserScanFragment&& other) noexcept {
  laser_scan_ = std::move(other.laser_scan_);
  occlusion_vector_ = std::move(other.occlusion_vector_);
  laser_scan_cloud_ = std::move(other.laser_scan_cloud_);

  initializeInternalContainer();

  return *this;
}

LaserScanFragment::LaserScanFragment(const LaserScanFragment& other, long first, long last) {
  if (first < 0 || first >= other.size()) {
    throw std::out_of_range("First index out of range. Index: " + std::to_string(first) +
        ". Container size: " + std::to_string(other.size()));
  }
  if (last < 0 || last > other.size()) {  // last == other.size() is valid, because it's other.end()
    throw std::out_of_range("Last index out of range. Index: " + std::to_string(first) +
        ". Container size: " + std::to_string(other.size()));
  }
  if (first >= last) {
    throw std::invalid_argument("First index needs to be less than last. First index: " + std::to_string(first) +
        ". Last index: " + std::to_string(last));
  }

  laser_scan_.header = other.getHeader();
  laser_scan_.angle_min = other.getAngleMin() + first * other.getAngleIncrement();
  laser_scan_.angle_max = other.getAngleMin() + (last - 1) * other.getAngleIncrement();
  laser_scan_.angle_increment = other.getAngleIncrement();
  laser_scan_.range_min = other.getRangeMin();
  laser_scan_.range_max = other.getRangeMax();
  laser_scan_.time_increment = other.laser_scan_.time_increment;
  laser_scan_.scan_time = other.laser_scan_.scan_time;
  laser_scan_.ranges.insert(laser_scan_.ranges.begin(),
                            other.laser_scan_.ranges.begin() + first,
                            other.laser_scan_.ranges.begin() + last);

  occlusion_vector_.insert(occlusion_vector_.begin(),
                           other.occlusion_vector_.begin() + first,
                           other.occlusion_vector_.begin() + last);

  laser_scan_cloud_.header = other.laser_scan_cloud_.header;
  laser_scan_cloud_.is_dense = other.laser_scan_cloud_.is_dense;
  laser_scan_cloud_.sensor_origin_ = other.laser_scan_cloud_.sensor_origin_;
  laser_scan_cloud_.sensor_orientation_ = other.laser_scan_cloud_.sensor_orientation_;
  laser_scan_cloud_.insert(laser_scan_cloud_.begin(),
                           other.laser_scan_cloud_.begin() + first,
                           other.laser_scan_cloud_.begin() + last);

  initializeInternalContainer();
}

LaserScanFragment::Iterator LaserScanFragment::begin() {
  return elements_.begin();
}

LaserScanFragment::ConstIterator LaserScanFragment::begin() const {
  return elements_.begin();
}

LaserScanFragment::ConstIterator LaserScanFragment::cbegin() const {
  return elements_.cbegin();
}

LaserScanFragment::Iterator LaserScanFragment::end() {
  return elements_.end();
}

LaserScanFragment::ConstIterator LaserScanFragment::end() const {
  return elements_.end();
}

LaserScanFragment::ConstIterator LaserScanFragment::cend() const {
  return elements_.cend();
}

LaserScanFragment::Reference LaserScanFragment::at(size_t index) {
  return elements_.at(index);
}

LaserScanFragment::ConstReference LaserScanFragment::at(size_t index) const {
  return elements_.at(index);
}

LaserScanFragment::Reference LaserScanFragment::operator[](size_t index) {
  return elements_[index];
}

LaserScanFragment::ConstReference LaserScanFragment::operator[](size_t index) const {
  return elements_[index];
}

FragmentElement& LaserScanFragment::front() {
  return elements_.front();
}

const FragmentElement& LaserScanFragment::front() const {
  return elements_.front();
}

FragmentElement& LaserScanFragment::back() {
  return elements_.back();
}

const FragmentElement& LaserScanFragment::back() const {
  return elements_.back();
}

bool LaserScanFragment::isValid() const {
  return std::all_of(elements_.cbegin(), elements_.cend(), [](const auto& el) {
    return el.isValid();
  });
}

void LaserScanFragment::initializeInternalContainer() {
  elements_.clear();

  elements_.reserve(laser_scan_.ranges.size());
  for (int i = 0; i < laser_scan_.ranges.size(); ++i) {
    elements_.push_back({
                            laser_scan_.angle_min + i * laser_scan_.angle_increment,
                            laser_scan_.ranges[i],
                            occlusion_vector_[i],
                            laser_scan_cloud_[i],
                            laser_scan_.ranges[i] < getRangeMin(),
                            laser_scan_.ranges[i] >= getRangeMax()
                        });
  }
}
}  // namespace data_types
}  // namespace laser_object_tracker
