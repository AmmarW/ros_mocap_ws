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

#include <mocap_optitrack/frame_statistics.h>

using mocap_optitrack::FrameStatistics;

namespace
{
double const kPeriod = 1.0 / 120.0;
}


// A stream with no gaps in the counter has lost nothing, and reports the rate
// the frames were captured at.
TEST(FrameStatistics, CleanStreamReportsNoLoss)
{
  FrameStatistics stats;
  for (int i = 0; i < 600; ++i)
  {
    stats.update(i, i * kPeriod, true, 0.001);
  }

  EXPECT_EQ(600, stats.getReceivedFrames());
  EXPECT_EQ(0, stats.getDroppedFrames());
  EXPECT_DOUBLE_EQ(0.0, stats.getDropPercent());
  EXPECT_NEAR(120.0, stats.getCaptureRateHz(), 1e-6);
  EXPECT_NEAR(120.0, stats.getReceivedRateHz(), 1e-6);
}


// A gap in the counter is a frame that was captured but never arrived. It has
// to be counted, because nothing else in the data records its absence.
TEST(FrameStatistics, CounterGapsAreCountedAsLoss)
{
  FrameStatistics stats;
  int frame = 0;
  for (int i = 0; i < 600; ++i)
  {
    stats.update(frame, frame * kPeriod, true, 0.001);
    frame += (i % 10 == 9) ? 2 : 1;   // drop every tenth
  }

  // 60 gaps are introduced but the last falls after the final update, so 59
  // are ever observed.
  EXPECT_EQ(600, stats.getReceivedFrames());
  EXPECT_EQ(59, stats.getDroppedFrames());
  EXPECT_NEAR(100.0 * 59 / 659, stats.getDropPercent(), 1e-9);
  EXPECT_EQ(1, stats.getLargestGap());
}


// Loss must not depress the reported capture rate: the system is still running
// at its configured rate, this node is just not seeing all of it.
TEST(FrameStatistics, CaptureRateIsUnaffectedByLossButReceivedRateIsNot)
{
  FrameStatistics stats;
  int frame = 0;
  for (int i = 0; i < 600; ++i)
  {
    stats.update(frame, frame * kPeriod, true, 0.001);
    frame += (i % 4 == 3) ? 2 : 1;    // lose a quarter
  }

  EXPECT_NEAR(120.0, stats.getCaptureRateHz(), 1e-6);
  EXPECT_LT(stats.getReceivedRateHz(), 100.0);
}


TEST(FrameStatistics, LargestGapReportsTheWorstBurst)
{
  FrameStatistics stats;
  stats.update(0, 0.0, true, 0.0);
  stats.update(1, kPeriod, true, 0.0);
  stats.update(40, 40 * kPeriod, true, 0.0);    // 38 missing
  stats.update(43, 43 * kPeriod, true, 0.0);    // 2 missing

  EXPECT_EQ(38 + 2, stats.getDroppedFrames());
  EXPECT_EQ(38, stats.getLargestGap());
}


// Untracked frames are not lost frames: they arrived, they just carry no pose.
// Conflating the two would hide whichever is actually happening.
TEST(FrameStatistics, UntrackedFramesAreCountedSeparatelyFromLoss)
{
  FrameStatistics stats;
  for (int i = 0; i < 100; ++i)
  {
    stats.update(i, i * kPeriod, i >= 25, 0.002);
  }

  EXPECT_EQ(0, stats.getDroppedFrames());
  EXPECT_EQ(25, stats.getUntrackedFrames());
  EXPECT_NEAR(25.0, stats.getUntrackedPercent(), 1e-9);
}


TEST(FrameStatistics, MarkerErrorIsSummarised)
{
  FrameStatistics stats;
  stats.update(0, 0.0, true, 0.001);
  stats.update(1, kPeriod, true, 0.003);
  stats.update(2, 2 * kPeriod, true, 0.002);

  EXPECT_NEAR(0.002, stats.getMeanMarkerError(), 1e-9);
  EXPECT_NEAR(0.003, stats.getMaxMarkerError(), 1e-9);
}


// A server restart sends the counter backwards. That is not a gap of minus a
// billion frames, so statistics must restart rather than report nonsense.
TEST(FrameStatistics, CounterGoingBackwardsRestartsRatherThanReportingLoss)
{
  FrameStatistics stats;
  for (int i = 1000; i < 1100; ++i)
  {
    stats.update(i, i * kPeriod, true, 0.001);
  }
  ASSERT_EQ(0, stats.getDroppedFrames());

  stats.update(0, 0.0, true, 0.001);   // restart
  stats.update(1, kPeriod, true, 0.001);

  EXPECT_EQ(0, stats.getDroppedFrames());
  EXPECT_EQ(2, stats.getReceivedFrames());
}


TEST(FrameStatistics, ResetClearsEverything)
{
  FrameStatistics stats;
  stats.update(0, 0.0, false, 0.5);
  stats.update(5, 5 * kPeriod, false, 0.5);
  ASSERT_GT(stats.getDroppedFrames(), 0);

  stats.reset();

  EXPECT_FALSE(stats.hasData());
  EXPECT_EQ(0, stats.getReceivedFrames());
  EXPECT_EQ(0, stats.getDroppedFrames());
  EXPECT_EQ(0, stats.getUntrackedFrames());
  EXPECT_DOUBLE_EQ(0.0, stats.getMaxMarkerError());
  EXPECT_DOUBLE_EQ(0.0, stats.getCaptureRateHz());
}


// Nothing may divide by zero before enough frames have arrived to divide by.
TEST(FrameStatistics, IsSafeBeforeAnyDataArrives)
{
  FrameStatistics stats;

  EXPECT_FALSE(stats.hasData());
  EXPECT_DOUBLE_EQ(0.0, stats.getCaptureRateHz());
  EXPECT_DOUBLE_EQ(0.0, stats.getReceivedRateHz());
  EXPECT_DOUBLE_EQ(0.0, stats.getDropPercent());
  EXPECT_DOUBLE_EQ(0.0, stats.getUntrackedPercent());
  EXPECT_DOUBLE_EQ(0.0, stats.getMeanMarkerError());

  stats.update(7, 1.0, true, 0.001);   // a single frame spans no time
  EXPECT_FALSE(stats.hasData());
  EXPECT_DOUBLE_EQ(0.0, stats.getCaptureRateHz());
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
