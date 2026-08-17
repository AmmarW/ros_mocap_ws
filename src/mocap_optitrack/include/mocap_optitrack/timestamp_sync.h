/*
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
 * Copyright (c) 2012, Clearpath Robotics, Inc., Alex Bencz
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
#ifndef __MOCAP_OPTITRACK_TIMESTAMP_SYNC_H__
#define __MOCAP_OPTITRACK_TIMESTAMP_SYNC_H__

#include <algorithm>

namespace mocap_optitrack
{

/// \brief Maps Motive's frame timestamp onto the ROS clock.
///
/// Motive stamps every frame with the instant the cameras captured it. The
/// driver's own arrival time is not a usable substitute: the socket is drained
/// by a polling loop, so a backlog is emitted in a burst and consecutive frames
/// pick up arrival times far closer together than the true capture interval.
/// Motive's timestamp does not suffer from that, but it counts seconds since
/// Motive started rather than since the epoch, so it needs an offset to reach
/// ROS time.
///
/// Transport delay is strictly positive, so of all samples seen, the one with
/// the smallest (rosNow - mocapTimestamp) is the least delayed and gives the
/// best estimate of the true offset. Estimation runs in two phases:
///
///  - Calibration: for the first calibrationSamples frames the running minimum
///    is adopted directly, so an unlucky first sample (arrival delays of several
///    hundred ms do occur at startup) is corrected out quickly.
///  - Tracking: afterwards the offset may only creep, at driftRateLimit
///    seconds per second. That is enough to follow genuine crystal drift between
///    the two hosts while ignoring per-frame jitter and stalls entirely.
///
/// The consequence that matters downstream: after calibration the interval
/// between successive published stamps reproduces the true capture interval to
/// within driftRateLimit, so differentiating the pose is physically meaningful.
///
/// Note on resolution: times are handled as doubles carrying a full UNIX epoch,
/// which leaves roughly 0.2 us of representable precision. That is four orders
/// of magnitude below a 120 Hz frame interval and so irrelevant here, but it
/// does mean published intervals are exact only to about a microsecond.
class TimestampSynchronizer
{
public:
  /// \param calibrationSamples Frames spent adopting the running minimum before
  ///        the offset is locked to drift-rate tracking. At 120 Hz the default
  ///        is approximately two seconds.
  /// \param driftRateLimit Maximum rate, in seconds per second, at which the
  ///        offset may move once calibrated. The default of 100 ppm comfortably
  ///        exceeds the drift between two ordinary crystal oscillators while
  ///        distorting the frame interval by only 0.01%.
  explicit TimestampSynchronizer(int calibrationSamples = 240,
                                 double driftRateLimit = 1.0e-4)
    : calibrationSamples(calibrationSamples),
      driftRateLimit(driftRateLimit),
      sampleCount(0),
      offsetEstimate(0.0),
      lastMocapTime(0.0),
      lastRosTime(0.0),
      initialized(false)
  {
  }

  /// \brief Convert one Motive frame timestamp into a ROS timestamp.
  /// \param mocapTimestamp Seconds since Motive started, from the data frame.
  /// \param rosNow Current ROS time, used only to estimate the offset.
  /// \return The capture instant expressed in ROS time.
  double toRosTime(double mocapTimestamp, double rosNow)
  {
    double const delta = rosNow - mocapTimestamp;

    // First frame, or Motive restarted and its clock went backwards.
    if (!initialized || mocapTimestamp < lastMocapTime)
    {
      reset();
      offsetEstimate = delta;
      initialized = true;
    }
    else if (sampleCount < calibrationSamples)
    {
      // Still calibrating: adopt any smaller offset immediately.
      offsetEstimate = std::min(offsetEstimate, delta);
    }
    else
    {
      // Calibrated: the offset may only creep, bounded by the drift rate, so
      // neither a stalled frame nor a burst can pull the timeline around.
      double const elapsed = mocapTimestamp - lastMocapTime;
      double const bound = driftRateLimit * elapsed;
      offsetEstimate = std::max(offsetEstimate - bound,
                                std::min(delta, offsetEstimate + bound));
    }

    ++sampleCount;
    lastMocapTime = mocapTimestamp;

    // Published stamps must not go backwards even while the offset is settling.
    double stamp = mocapTimestamp + offsetEstimate;
    if (initialized && sampleCount > 1 && stamp <= lastRosTime)
    {
      stamp = lastRosTime;
    }
    lastRosTime = stamp;
    return stamp;
  }

  /// \brief True once at least one frame has been seen.
  bool isInitialized() const
  {
    return initialized;
  }

  /// \brief True once the offset has locked to drift-rate tracking.
  bool isCalibrated() const
  {
    return initialized && sampleCount >= calibrationSamples;
  }

  /// \brief Current estimate of (ROS time - Motive time), in seconds.
  double getOffset() const
  {
    return offsetEstimate;
  }

  void reset()
  {
    sampleCount = 0;
    offsetEstimate = 0.0;
    lastMocapTime = 0.0;
    lastRosTime = 0.0;
    initialized = false;
  }

private:
  int calibrationSamples;
  double driftRateLimit;

  int sampleCount;
  double offsetEstimate;
  double lastMocapTime;
  double lastRosTime;
  bool initialized;
};

}  // namespace mocap_optitrack

#endif  // __MOCAP_OPTITRACK_TIMESTAMP_SYNC_H__
