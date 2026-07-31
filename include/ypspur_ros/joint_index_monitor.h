/*
 * Copyright (c) 2026, the ypspur_ros authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef YPSPUR_ROS_JOINT_INDEX_MONITOR_H
#define YPSPUR_ROS_JOINT_INDEX_MONITOR_H

#include <cmath>
#include <limits>

namespace ypspur_ros
{
// Detects loss of the joint index signal by checking that the index signal
// is observed at least once while the joint travels the given angle.
class JointIndexMonitor
{
public:
  explicit JointIndexMonitor(
      const double travel_threshold = std::numeric_limits<double>::infinity())
    : travel_threshold_(travel_threshold)
    , initialized_(false)
    , error_(false)
    , wang_time_last_(0)
    , ref_angle_(0)
  {
  }

  void update(const double wang, const double wang_time)
  {
    if (!initialized_)
    {
      wang_time_last_ = wang_time;
      ref_angle_ = wang;
      initialized_ = true;
      return;
    }
    if (wang_time != wang_time_last_)
    {
      wang_time_last_ = wang_time;
      ref_angle_ = wang;
      error_ = false;
      return;
    }
    if (std::abs(wang - ref_angle_) > travel_threshold_)
    {
      error_ = true;
    }
  }

  bool hasError() const
  {
    return error_;
  }

private:
  double travel_threshold_;
  bool initialized_;
  bool error_;
  double wang_time_last_;
  double ref_angle_;
};
}  // namespace ypspur_ros

#endif  // YPSPUR_ROS_JOINT_INDEX_MONITOR_H
