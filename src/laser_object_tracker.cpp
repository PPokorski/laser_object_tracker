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

int main(int ac, char** av) {
    sensor_msgs::LaserScan ls;
    ls.angle_min = - 3.14;
    ls.angle_max = 3.14;
    ls.range_max = 10.0;
    ls.angle_increment = 1.57;
    ls.ranges = {1.0, 2.0, 3.0, 4.0, 5.0};
    laser_object_tracker::data_types::LaserScanFragment::LaserScanFragmentFactory factory;
    auto fragment = factory.fromLaserScan(std::move(ls));

    for (auto it = fragment.begin(); it != fragment.end(); ++it)
    {
        it->is_occluded_ = true;
        std::cout << std::boolalpha;
        std::cout << "Angle: " << it->angle_  << " Range: " << it->range_ << " Is occluded: " << it->is_occluded_ << std::endl;
        std::cout << "PointXYZ: [" << it->point_.x << ", " << it->point_.y << ", " << it->point_.z << "]" << std::endl;
    }

    return 0;
}
