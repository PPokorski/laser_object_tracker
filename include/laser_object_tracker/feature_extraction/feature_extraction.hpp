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

#ifndef LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURE_EXTRACTION_HPP
#define LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURE_EXTRACTION_HPP

#include "laser_object_tracker/feature_extraction/features/features.hpp"
#include "laser_object_tracker/feature_extraction/pcl/sac_model_cross2d.hpp"
#include "laser_object_tracker/feature_extraction/base_feature_extraction.hpp"
#include "laser_object_tracker/feature_extraction/multi_line_detection.hpp"
#include "laser_object_tracker/feature_extraction/occlusion_checking/occlusion_checking.hpp"
#include "laser_object_tracker/feature_extraction/random_sample_consensus_corner_detection.hpp"
#include "laser_object_tracker/feature_extraction/random_sample_consensus_segment_detection.hpp"
#include "laser_object_tracker/feature_extraction/search_based_corner_detection.hpp"

#endif  // LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURE_EXTRACTION_HPP
