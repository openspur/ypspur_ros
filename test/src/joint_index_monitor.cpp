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

#include <ypspur_ros/joint_index_monitor.h>

#include <gtest/gtest.h>

namespace
{
constexpr double kThreshold = 6.4;
}  // namespace

TEST(JointIndexMonitor, NeverErrorWhenDefaultConstructed)
{
  ypspur_ros::JointIndexMonitor monitor;
  double wang = 0;
  for (int i = 0; i < 1000; ++i)
  {
    wang += 1.0;
    monitor.update(wang, 0);
    ASSERT_FALSE(monitor.hasError()) << "wang: " << wang;
  }
}

TEST(JointIndexMonitor, NoErrorWhileStopped)
{
  ypspur_ros::JointIndexMonitor monitor(kThreshold);
  for (int i = 0; i < 100; ++i)
  {
    monitor.update(1.0, 0);
    ASSERT_FALSE(monitor.hasError());
  }
}

TEST(JointIndexMonitor, FirstSampleBecomesReference)
{
  // Non-zero initial wang_time must be treated as the reference, not as an
  // index signal
  ypspur_ros::JointIndexMonitor monitor(kThreshold);
  monitor.update(10.0, 123.4);
  ASSERT_FALSE(monitor.hasError());
  monitor.update(10.0 + kThreshold + 0.1, 123.4);
  ASSERT_TRUE(monitor.hasError());
}

TEST(JointIndexMonitor, ErrorOnForwardTravelWithoutIndex)
{
  ypspur_ros::JointIndexMonitor monitor(kThreshold);
  double wang = 0;
  while (wang < kThreshold)
  {
    monitor.update(wang, 0);
    ASSERT_FALSE(monitor.hasError()) << "wang: " << wang;
    wang += 0.1;
  }
  monitor.update(kThreshold + 0.1, 0);
  ASSERT_TRUE(monitor.hasError());
}

TEST(JointIndexMonitor, ErrorOnReverseTravelWithoutIndex)
{
  ypspur_ros::JointIndexMonitor monitor(kThreshold);
  double wang = 0;
  while (wang > -kThreshold)
  {
    monitor.update(wang, 0);
    ASSERT_FALSE(monitor.hasError()) << "wang: " << wang;
    wang -= 0.1;
  }
  monitor.update(-kThreshold - 0.1, 0);
  ASSERT_TRUE(monitor.hasError());
}

TEST(JointIndexMonitor, NoErrorOnOscillationWithinThreshold)
{
  ypspur_ros::JointIndexMonitor monitor(kThreshold);
  for (int cycle = 0; cycle < 10; ++cycle)
  {
    for (double wang = 0; wang < kThreshold * 0.9; wang += 0.1)
    {
      monitor.update(wang, 0);
      ASSERT_FALSE(monitor.hasError());
    }
    for (double wang = kThreshold * 0.9; wang > 0; wang -= 0.1)
    {
      monitor.update(wang, 0);
      ASSERT_FALSE(monitor.hasError());
    }
  }
}

TEST(JointIndexMonitor, NoErrorWithPeriodicIndex)
{
  ypspur_ros::JointIndexMonitor monitor(kThreshold);
  double wang = 0;
  double wang_time = 100.0;
  monitor.update(wang, wang_time);
  for (int rev = 0; rev < 10; ++rev)
  {
    for (int i = 0; i < 63; ++i)
    {
      wang += 0.1;
      monitor.update(wang, wang_time);
      ASSERT_FALSE(monitor.hasError()) << "wang: " << wang;
    }
    wang_time += 6.3;
  }
}

TEST(JointIndexMonitor, RecoverOnIndexSignal)
{
  ypspur_ros::JointIndexMonitor monitor(kThreshold);
  monitor.update(0, 0);
  monitor.update(kThreshold + 0.1, 0);
  ASSERT_TRUE(monitor.hasError());

  // Index signal resets the error and the reference angle
  monitor.update(kThreshold + 0.2, 200.0);
  ASSERT_FALSE(monitor.hasError());
  monitor.update(kThreshold + 0.2 + kThreshold * 0.9, 200.0);
  ASSERT_FALSE(monitor.hasError());
  monitor.update(kThreshold + 0.2 + kThreshold + 0.1, 200.0);
  ASSERT_TRUE(monitor.hasError());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
