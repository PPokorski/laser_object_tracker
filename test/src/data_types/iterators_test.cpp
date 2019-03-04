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

#include <gtest/gtest.h>

#include "laser_object_tracker/data_types/fragment_iterator.hpp"

#include "test/utils.hpp"
#include "test/data_types/test_data.hpp"

template <class IteratorT>
class FragmentIteratorTest : public testing::Test {
};

using IteratorTypes = testing::Types<laser_object_tracker::data_types::FragmentIterator,
                                     laser_object_tracker::data_types::ConstFragmentIterator>;
TYPED_TEST_CASE(FragmentIteratorTest, IteratorTypes);

TYPED_TEST(FragmentIteratorTest, InitializationWithIteratorsTest) {
    auto reference = test::getFragmentUnique2();

    TypeParam iterator(0.0, 1.0,
            reference.laser_scan_.ranges.begin(),
            reference.occlusion_array_.begin(),
            reference.laser_scan_cloud_.begin());

    // Operator*
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
            0.0, (*iterator).getAngle());
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
            reference.laser_scan_.ranges.front(), (*iterator).range());
    EXPECT_EQ(reference.occlusion_array_.front(), (*iterator).isOccluded());
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::compare(lhs, rhs);},
                 reference.laser_scan_cloud_.front(), (*iterator).point());

    // Operator->
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
                 0.0, iterator->getAngle());
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
                 reference.laser_scan_.ranges.front(), iterator->range());
    EXPECT_EQ(reference.occlusion_array_.front(), iterator->isOccluded());
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::compare(lhs, rhs);},
                 reference.laser_scan_cloud_.front(), iterator->point());
}

TYPED_TEST(FragmentIteratorTest, InitializationCopyTest) {
    auto reference = test::getFragmentUnique2();

    TypeParam iterator(0.0, 1.0,
            reference.laser_scan_.ranges.begin(),
            reference.occlusion_array_.begin(),
            reference.laser_scan_cloud_.begin());

    TypeParam iterator2(iterator);

    EXPECT_EQ(iterator, iterator2);
    ++iterator2;
    EXPECT_NE(iterator, iterator2);
}

TYPED_TEST(FragmentIteratorTest, CopyAssignmentTest) {
    auto reference = test::getFragmentUnique2();

    TypeParam iterator(0.0, 1.0,
            reference.laser_scan_.ranges.begin(),
            reference.occlusion_array_.begin(),
            reference.laser_scan_cloud_.begin());

    TypeParam iterator2;
    iterator2 = iterator;

    EXPECT_EQ(iterator, iterator2);
    ++iterator2;
    EXPECT_NE(iterator, iterator2);
}

TYPED_TEST(FragmentIteratorTest, ComparisionTest) {
    auto reference = test::getFragmentUnique2();

    TypeParam iterator(0.0, 1.0,
            reference.laser_scan_.ranges.begin(),
            reference.occlusion_array_.begin(),
            reference.laser_scan_cloud_.begin());

    // Test for equality, begin iterators
    EXPECT_EQ(iterator, TypeParam(0.0, 1.0,
                                  reference.laser_scan_.ranges.begin(),
                                  reference.occlusion_array_.begin(),
                                  reference.laser_scan_cloud_.begin()));

    EXPECT_EQ(iterator, TypeParam(4.0, 3.0,
                                  reference.laser_scan_.ranges.begin(),
                                  reference.occlusion_array_.begin(),
                                  reference.laser_scan_cloud_.begin()));

    // Test for equality, ++begin iterators
    TypeParam iterator2(0.0, 1.0,
            ++reference.laser_scan_.ranges.begin(),
            ++reference.occlusion_array_.begin(),
            ++reference.laser_scan_cloud_.begin());

    EXPECT_EQ(iterator2, TypeParam(0.0, 1.0,
                                   ++reference.laser_scan_.ranges.begin(),
                                   ++reference.occlusion_array_.begin(),
                                   ++reference.laser_scan_cloud_.begin()));

    // Test for inequality
    // Increment laser scan iterator
    EXPECT_NE(iterator, TypeParam(0.0, 1.0,
                                  ++reference.laser_scan_.ranges.begin(),
                                  reference.occlusion_array_.begin(),
                                  reference.laser_scan_cloud_.begin()));

    // Increment occlusion iterator
    EXPECT_NE(iterator, TypeParam(0.0, 1.0,
                                  reference.laser_scan_.ranges.begin(),
                                  ++reference.occlusion_array_.begin(),
                                  reference.laser_scan_cloud_.begin()));

    // Increment point cloud iterator
    EXPECT_NE(iterator, TypeParam(0.0, 1.0,
                                  reference.laser_scan_.ranges.begin(),
                                  reference.occlusion_array_.begin(),
                                  ++reference.laser_scan_cloud_.begin()));

    // Increment laser scan and occlusion iterator
    EXPECT_NE(iterator, TypeParam(0.0, 1.0,
                                  ++reference.laser_scan_.ranges.begin(),
                                  ++reference.occlusion_array_.begin(),
                                  reference.laser_scan_cloud_.begin()));

    // Increment laser scan and point cloud iterator
    EXPECT_NE(iterator, TypeParam(0.0, 1.0,
                                  ++reference.laser_scan_.ranges.begin(),
                                  reference.occlusion_array_.begin(),
                                  ++reference.laser_scan_cloud_.begin()));

    // Increment occlusion and point cloud iterator
    EXPECT_NE(iterator, TypeParam(0.0, 1.0,
                                  reference.laser_scan_.ranges.begin(),
                                  ++reference.occlusion_array_.begin(),
                                  ++reference.laser_scan_cloud_.begin()));
}

TYPED_TEST(FragmentIteratorTest, IncrementingTest) {
    auto reference = test::getFragmentUnique2();

    // Pre incrementation
    TypeParam iterator(0.0, 1.0,
            reference.laser_scan_.ranges.begin(),
            reference.occlusion_array_.begin(),
            reference.laser_scan_cloud_.begin());

    ++iterator;
    EXPECT_EQ(iterator, TypeParam(0.0, 1.0,
                                  ++reference.laser_scan_.ranges.begin(),
                                  ++reference.occlusion_array_.begin(),
                                  ++reference.laser_scan_cloud_.begin()));
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
                 1.0, iterator->getAngle());

    ++iterator;
    EXPECT_EQ(iterator, TypeParam(0.0, 1.0,
                                  ++ ++reference.laser_scan_.ranges.begin(),
                                  ++ ++reference.occlusion_array_.begin(),
                                  ++ ++reference.laser_scan_cloud_.begin()));
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
                 2.0, iterator->getAngle());


    // Post incrementation
    TypeParam iterator2(0.0, 1.0,
            reference.laser_scan_.ranges.begin(),
            reference.occlusion_array_.begin(),
            reference.laser_scan_cloud_.begin());

    TypeParam iterator3 = iterator2++;
    EXPECT_EQ(iterator2, ++iterator3);
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
                 1.0, iterator3->getAngle());

    iterator3 = iterator2++;
    EXPECT_EQ(iterator2, ++iterator3);
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
                 2.0, iterator3->getAngle());
}

TYPED_TEST(FragmentIteratorTest, DecrementingTest) {
    auto reference = test::getFragmentUnique2();

    // Pre decrementation
    TypeParam iterator(0.0, 1.0,
            reference.laser_scan_.ranges.end(),
            reference.occlusion_array_.end(),
            reference.laser_scan_cloud_.end());

    --iterator;
    EXPECT_EQ(iterator, TypeParam(0.0, 1.0,
                                  --reference.laser_scan_.ranges.end(),
                                  --reference.occlusion_array_.end(),
                                  --reference.laser_scan_cloud_.end()));
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
                 -1.0, iterator->getAngle());

    --iterator;
    EXPECT_EQ(iterator, TypeParam(0.0, 1.0,
                                  -- --reference.laser_scan_.ranges.end(),
                                  -- --reference.occlusion_array_.end(),
                                  -- --reference.laser_scan_cloud_.end()));
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
                 -2.0, iterator->getAngle());


    // Post decrementation
    TypeParam iterator2(0.0, 1.0,
            reference.laser_scan_.ranges.end(),
            reference.occlusion_array_.end(),
            reference.laser_scan_cloud_.end());

    TypeParam iterator3 = iterator2--;
    EXPECT_EQ(iterator2, --iterator3);
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
                 -1.0, iterator3->getAngle());

    iterator3 = iterator2--;
    EXPECT_EQ(iterator2, --iterator3);
    EXPECT_PRED2([](const auto& lhs, const auto& rhs) {return test::close(lhs, rhs);},
                 -2.0, iterator3->getAngle());
}
