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

#ifndef LASER_OBJECT_TRACKER_FRAGMENT_ITERATOR_HPP
#define LASER_OBJECT_TRACKER_FRAGMENT_ITERATOR_HPP

// STD
#include <iterator>

// PROJECT
#include "laser_object_tracker/data_types/definitions.hpp"

namespace laser_object_tracker {
namespace data_types {
namespace internal {

/**
 * @brief Helper class defining common interface for const and mutable iterator
 * FragmentIteratorT is a proxy, zip-like bidirectional iterator, zipping
 * LaserScanType::_ranges_type iterator, OcclusionType iterator, PointCloudType iterator into a single iterator.
 * @tparam is_const Specializes for mutable and const iterator
 */
template<bool is_const>
class FragmentIteratorT {
 public:
    // Iterator traits
    using self_type = FragmentIteratorT<is_const>;
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = FragmentElement;
    using difference_type = long;
    using pointer = std::unique_ptr<FragmentElement>;
    // Return value for operator->, specializes for const and mutable iterator
    using ret_pointer = typename std::conditional_t<is_const,
                                                    std::unique_ptr<const FragmentElement>,
                                                    std::unique_ptr<FragmentElement>>;
    using reference = FragmentElement&;
    // Return value for operator*, specializes for const and mutable iterator
    using ret_reference = typename std::conditional_t<is_const,
                                                      const FragmentElement&,
                                                      FragmentElement&>;

    // Underyling iterators, specializes for const and mutable iterator
    using LaserScanIterator = typename std::conditional_t<is_const,
                                                          LaserScanType::_ranges_type::const_iterator,
                                                          LaserScanType::_ranges_type::iterator>;
    using OcclusionIterator = typename std::conditional_t<is_const,
                                                          OcclusionType::const_iterator,
                                                          OcclusionType::iterator>;
    using PointCloudIterator = typename std::conditional_t<is_const,
                                                           PointCloudType::const_iterator,
                                                           PointCloudType::iterator>;

    /**
     * @brief Default c-tor
     */
    FragmentIteratorT() = default;

    /**
     * @brief Basic c-tor for iterator initialization
     * @param angle Initialization value for underlying polar coordinates
     * @param angle_increment How much angle_ will change when incrementing or decrementing
     * @param range_iterator Iterator for ranges (measurements of laser scanner)
     * @param occlusion_iterator Iterator for occlusion (whether or not the measurement is occluded)
     * @param point_iterator Iterator for points (cartesian coordinates of measurement)
     */
    FragmentIteratorT(double angle, double angle_increment,
            const LaserScanIterator& range_iterator,
            const OcclusionIterator& occlusion_iterator,
            const PointCloudIterator& point_iterator) :
            angle_(angle),
            angle_increment_(angle_increment),
            range_iterator_(range_iterator),
            occlusion_iterator_(occlusion_iterator),
            point_iterator_(point_iterator) {
        resetPtr();
    }

    /**
     * @brief Copy c-tor
     * @param other
     */
    FragmentIteratorT(const self_type& other) :
            angle_(other.angle_),
            angle_increment_(other.angle_increment_),
            range_iterator_(other.range_iterator_),
            occlusion_iterator_(other.occlusion_iterator_),
            point_iterator_(other.point_iterator_) {
        resetPtr();
    }

    /**
     * @brief Copy assignment operator
     * @param other
     * @return *this
     */
    self_type& operator=(const self_type& other) {
        angle_ = other.angle_;
        angle_increment_ = other.angle_increment_;
        range_iterator_ = other.range_iterator_;
        occlusion_iterator_ = other.occlusion_iterator_;
        point_iterator_ = other.point_iterator_;

        resetPtr();
        return *this;
    }

    /**
     * @brief Pre incrementation. Increments underlying iterators and angle.
     * @return
     */
    self_type& operator++() {
        angle_ += angle_increment_;
        ++range_iterator_;
        ++occlusion_iterator_;
        ++point_iterator_;

        resetPtr();
        return *this;
    }

    /**
     * @brief Post incrementation. Increments underlying iterators and angle.
     * @return
     */
    self_type operator++(int) {
        self_type tmp = *this;
        ++(*this);
        return tmp;
    }

    /**
     * @brief Pre decrementation. Decrements underlying iterators and angle.
     * @return
     */
    self_type& operator--() {
        angle_ -= angle_increment_;
        --range_iterator_;
        --occlusion_iterator_;
        --point_iterator_;

        resetPtr();
        return *this;
    }

    /**
     * @brief Post decrementation. Decrements underlying iterators and angle.
     * @return
     */
    self_type operator--(int) {
        self_type tmp = *this;
        --(*this);
        return tmp;
    }

    /**
     * @brief Dereferencing operator. Returns proxy object of type ret_reference.
     * @return Proxy object of type ret_reference
     */
    ret_reference operator*() const {
        return *ptr_;
    }

    /**
     * @brief Arrow operator. Returns proxy object of type ret_pointer.
     * @return Proxy object of type ret_pointer
     */
    const ret_pointer& operator->() const {
        return ptr_;
    }

    /**
     * @brief Comparision operator.
     * @note The operator checks only equality of underlying iterators, not an angle.
     * If iterators differ only in angle, then they will be considered same
     * @param rhs
     * @return True when underlying operators are equal, false otherwise
     */
    bool operator==(const self_type& rhs) const {
        return range_iterator_ == rhs.range_iterator_ &&
               occlusion_iterator_ == rhs.occlusion_iterator_ &&
               point_iterator_ == rhs.point_iterator_;
    }

    /**
     * @brief Comparision operator.
     * @note The operator checks only equality of underlying iterators, not an angle.
     * If iterators differ only in angle, then they will be considered same
     * @param rhs
     * @return False when underlying operators are equal, true otherwise
     */
    bool operator!=(const self_type& rhs) const {
        return !operator==(rhs);
    }

 private:
    void resetPtr() {
        ptr_.reset(new FragmentElement{
                angle_,
                const_cast<LaserScanType::_ranges_type::reference>(*range_iterator_),
                const_cast<OcclusionType::reference>(*occlusion_iterator_),
                const_cast<PointCloudType::PointType&>(*point_iterator_)
        });
    }
    double angle_;
    double angle_increment_;
    LaserScanIterator range_iterator_;
    OcclusionIterator occlusion_iterator_;
    PointCloudIterator point_iterator_;

    ret_pointer ptr_;
};

}  // namespace internal

/**
 * @brief Mutable specialization of internal::FragmentIteratorT
 */
using FragmentIterator = internal::FragmentIteratorT<false>;
/**
 * @brief Const specialization of internal::FragmentIteratorT
 */
using ConstFragmentIterator = internal::FragmentIteratorT<true>;

}  // namespace data_types
}  // namespace laser_object_tracker

#endif  // LASER_OBJECT_TRACKER_FRAGMENT_ITERATOR_HPP
