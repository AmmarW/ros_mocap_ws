/*
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <mocap_optitrack/timestamp_sync.h>

using mocap_optitrack::TimestampSynchronizer;

namespace
{
double const kFramePeriod = 1.0 / 120.0;
double const kEpoch = 1762822941.0;   // arbitrary ROS-side epoch
int const kCalibration = 240;

// Times carry a full UNIX epoch in a double, which resolves to roughly 0.2 us.
// Assertions are made to 1 us: four orders of magnitude tighter than the frame
// interval under test, and comfortably above the representation floor.
double const kTolerance = 1e-6;

/// Replay a capture sequence through the synchronizer.
/// \param arrivalDelay Delay, per frame, between capture and the driver seeing
///        it. This is the quantity that arrival-time stamping gets wrong.
std::vector<double> replay(std::vector<double> const& captureTimes,
                           std::vector<double> const& arrivalDelay,
                           TimestampSynchronizer* sync)
{
  std::vector<double> out;
  out.reserve(captureTimes.size());
  for (size_t i = 0; i < captureTimes.size(); ++i)
  {
    double const rosNow = kEpoch + captureTimes[i] + arrivalDelay[i];
    out.push_back(sync->toRosTime(captureTimes[i], rosNow));
  }
  return out;
}
}  // namespace


// With a perfectly prompt link, published intervals equal capture intervals.
TEST(TimestampSynchronizer, RecoversUniformIntervalsOnACleanLink)
{
  int const n = 1200;
  std::vector<double> capture, delay;
  for (int i = 0; i < n; ++i)
  {
    capture.push_back(i * kFramePeriod);
    delay.push_back(0.0002);
  }

  TimestampSynchronizer sync(kCalibration);
  std::vector<double> stamps = replay(capture, delay, &sync);

  ASSERT_TRUE(sync.isCalibrated());
  for (size_t i = kCalibration + 1; i < stamps.size(); ++i)
  {
    EXPECT_NEAR(kFramePeriod, stamps[i] - stamps[i - 1], kTolerance)
      << "interval " << i << " drifted from the true capture period";
  }
}


// The defect this driver had: a stalled socket queue is drained in a burst, so
// arrival times bunch up. Capture times do not, and neither must our output.
TEST(TimestampSynchronizer, IsUnaffectedByStallAndBurstArrival)
{
  int const n = 1200;
  std::vector<double> capture, delay;
  for (int i = 0; i < n; ++i)
  {
    capture.push_back(i * kFramePeriod);
    // Every 200th frame the reader stalls for 300 ms; the frames captured
    // during the stall are then delivered back-to-back, each with a shrinking
    // delay. This reproduces the burst seen in the recorded bags.
    int const phase = i % 200;
    if (phase < 36 && i > 200)
    {
      delay.push_back(0.300 - phase * kFramePeriod);
    }
    else
    {
      delay.push_back(0.0002);
    }
  }

  TimestampSynchronizer sync(kCalibration);
  std::vector<double> stamps = replay(capture, delay, &sync);

  // Arrival intervals collapse to near zero inside a burst...
  double minArrivalInterval = 1.0;
  for (int i = 1; i < n; ++i)
  {
    double const arrivalInterval =
      (capture[i] + delay[i]) - (capture[i - 1] + delay[i - 1]);
    minArrivalInterval = std::min(minArrivalInterval, arrivalInterval);
  }
  EXPECT_LT(minArrivalInterval, 0.001)
    << "test fixture failed to reproduce the burst";

  // ...but published intervals must stay at the true capture period.
  for (size_t i = kCalibration + 1; i < stamps.size(); ++i)
  {
    EXPECT_NEAR(kFramePeriod, stamps[i] - stamps[i - 1], kTolerance)
      << "burst leaked into the published interval at " << i;
  }
}


// A dropped frame must widen the interval to a whole multiple of the period,
// which is what makes the loss visible to a consumer instead of silent.
TEST(TimestampSynchronizer, PreservesGapsWhenFramesAreDropped)
{
  std::vector<double> capture, delay;
  int frame = 0;
  for (int i = 0; i < 1200; ++i)
  {
    capture.push_back(frame * kFramePeriod);
    delay.push_back(0.0002);
    // drop every 50th frame
    frame += (i % 50 == 49) ? 2 : 1;
  }

  TimestampSynchronizer sync(kCalibration);
  std::vector<double> stamps = replay(capture, delay, &sync);

  int doubleGaps = 0;
  for (size_t i = kCalibration + 1; i < stamps.size(); ++i)
  {
    double const interval = stamps[i] - stamps[i - 1];
    double const frames = interval / kFramePeriod;
    double const nearestWholeFrames = std::floor(frames + 0.5);
    EXPECT_NEAR(nearestWholeFrames * kFramePeriod, interval, kTolerance)
      << "interval " << i << " is not a whole number of frames";
    if (frames > 1.5)
    {
      ++doubleGaps;
    }
  }
  EXPECT_GT(doubleGaps, 15) << "dropped frames were not preserved as gaps";
}


// An unlucky first sample (hundreds of ms late) must not permanently offset the
// stream; calibration exists to correct exactly that.
TEST(TimestampSynchronizer, RecoversFromALateFirstSample)
{
  int const n = 1200;
  std::vector<double> capture, delay;
  for (int i = 0; i < n; ++i)
  {
    capture.push_back(i * kFramePeriod);
    delay.push_back(i == 0 ? 0.728 : 0.0002);   // 728 ms, as seen in the bags
  }

  TimestampSynchronizer sync(kCalibration);
  std::vector<double> stamps = replay(capture, delay, &sync);

  // Once calibrated the offset should reflect the prompt samples, not the
  // late first one. The offset spans from Motive's clock to ROS time, so it
  // carries the epoch as well as the transport delay.
  EXPECT_NEAR(kEpoch + 0.0002, sync.getOffset(), kTolerance);

  double const expected = kEpoch + capture.back() + 0.0002;
  EXPECT_NEAR(expected, stamps.back(), kTolerance);
}


// Stamps must never move backwards, even while the offset is still settling.
TEST(TimestampSynchronizer, NeverEmitsNonMonotonicStamps)
{
  int const n = 1200;
  std::vector<double> capture, delay;
  for (int i = 0; i < n; ++i)
  {
    capture.push_back(i * kFramePeriod);
    // Decreasing delay across calibration is the case that would drag the
    // offset, and hence the stamps, backwards.
    delay.push_back(i < kCalibration ? 0.5 - i * 0.002 : 0.0002);
  }

  TimestampSynchronizer sync(kCalibration);
  std::vector<double> stamps = replay(capture, delay, &sync);

  // Strictly greater, not merely non-decreasing. Repeating the previous stamp
  // yields a zero interval, which breaks differentiation just as surely as a
  // negative one and would otherwise pass this test.
  int duplicates = 0;
  for (size_t i = 1; i < stamps.size(); ++i)
  {
    EXPECT_GT(stamps[i], stamps[i - 1])
      << "stamp failed to advance at " << i;
    if (stamps[i] == stamps[i - 1])
    {
      ++duplicates;
    }
  }
  EXPECT_EQ(0, duplicates) << "published stamps repeated, giving dt == 0";
}


// The late first sample is the case that used to produce repeated stamps: the
// offset shrank towards the running minimum faster than capture time advanced.
TEST(TimestampSynchronizer, LateFirstSampleDoesNotStallTheTimeline)
{
  int const n = 400;
  std::vector<double> capture, delay;
  for (int i = 0; i < n; ++i)
  {
    capture.push_back(i * kFramePeriod);
    delay.push_back(i == 0 ? 0.728 : 0.0002);
  }

  TimestampSynchronizer sync(kCalibration);
  std::vector<double> stamps = replay(capture, delay, &sync);

  for (size_t i = 1; i < stamps.size(); ++i)
  {
    EXPECT_GT(stamps[i] - stamps[i - 1], 0.0)
      << "interval " << i << " was not positive while the offset converged";
  }
}


// Restarting Motive resets its clock to zero; the synchronizer must re-acquire
// rather than emit a huge jump.
TEST(TimestampSynchronizer, ReacquiresAfterServerRestart)
{
  TimestampSynchronizer sync(10);

  for (int i = 0; i < 200; ++i)
  {
    sync.toRosTime(i * kFramePeriod, kEpoch + i * kFramePeriod + 0.0002);
  }
  ASSERT_TRUE(sync.isCalibrated());

  // Motive restarts: its timestamp goes back to near zero while ROS time
  // continues forward.
  double const rosAfterRestart = kEpoch + 500.0;
  double const stamp = sync.toRosTime(0.0, rosAfterRestart);

  EXPECT_FALSE(sync.isCalibrated()) << "restart should re-enter calibration";
  EXPECT_NEAR(rosAfterRestart, stamp, kTolerance);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
