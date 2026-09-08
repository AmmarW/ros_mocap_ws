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
#ifndef __MOCAP_OPTITRACK_FRAME_STATISTICS_H__
#define __MOCAP_OPTITRACK_FRAME_STATISTICS_H__

namespace mocap_optitrack
{

/// \brief Accumulates stream health over a measurement window.
///
/// Frame loss cannot be seen from the received data alone: a frame that never
/// arrives leaves nothing behind. The server's frame counter is what makes it
/// visible, because a gap in the counter is a frame that was captured but not
/// received. Counting those, and comparing the observed rate against the rate
/// the counter implies, distinguishes a healthy stream from one that is quietly
/// losing data.
///
/// Statistics accumulate until reset(), which a caller is expected to invoke
/// each time it has reported them, so each report covers one interval rather
/// than the whole session.
class FrameStatistics
{
public:
  FrameStatistics()
  {
    reset();
  }

  /// \brief Record one received frame.
  /// \param frameNumber The server's counter for this frame.
  /// \param captureTime Capture instant in seconds, on any consistent clock.
  /// \param trackingValid Whether any body in the frame was solved.
  /// \param markerError Mean marker error for the frame.
  void update(int frameNumber, double captureTime,
              bool trackingValid, double markerError)
  {
    if (!hasAny)
    {
      firstFrameNumber = frameNumber;
      firstCaptureTime = captureTime;
      hasAny = true;
    }
    else
    {
      int const advance = frameNumber - lastFrameNumber;

      // A counter that fails to advance, or moves backwards, means the server
      // restarted or the counter wrapped. Neither is frame loss, so begin
      // again rather than report a spurious gap.
      if (advance <= 0)
      {
        reset();
        firstFrameNumber = frameNumber;
        firstCaptureTime = captureTime;
        hasAny = true;
      }
      else if (advance > 1)
      {
        droppedFrames += advance - 1;
        if (advance - 1 > largestGap)
        {
          largestGap = advance - 1;
        }
      }
    }

    lastFrameNumber = frameNumber;
    lastCaptureTime = captureTime;
    ++receivedFrames;

    if (!trackingValid)
    {
      ++untrackedFrames;
    }
    markerErrorSum += markerError;
    if (markerError > markerErrorMax)
    {
      markerErrorMax = markerError;
    }
  }

  void reset()
  {
    hasAny = false;
    firstFrameNumber = 0;
    lastFrameNumber = 0;
    firstCaptureTime = 0.0;
    lastCaptureTime = 0.0;
    receivedFrames = 0;
    droppedFrames = 0;
    untrackedFrames = 0;
    largestGap = 0;
    markerErrorSum = 0.0;
    markerErrorMax = 0.0;
  }

  bool hasData() const
  {
    return hasAny && receivedFrames > 1;
  }

  int getReceivedFrames() const
  {
    return receivedFrames;
  }

  int getDroppedFrames() const
  {
    return droppedFrames;
  }

  int getLargestGap() const
  {
    return largestGap;
  }

  int getUntrackedFrames() const
  {
    return untrackedFrames;
  }

  /// \brief Frames lost as a percentage of those the counter says were captured.
  double getDropPercent() const
  {
    int const expected = receivedFrames + droppedFrames;
    if (expected <= 0)
    {
      return 0.0;
    }
    return 100.0 * droppedFrames / expected;
  }

  /// \brief Frames without a solved body, as a percentage of those received.
  double getUntrackedPercent() const
  {
    if (receivedFrames <= 0)
    {
      return 0.0;
    }
    return 100.0 * untrackedFrames / receivedFrames;
  }

  /// \brief Rate at which frames were captured, counting those that were lost.
  ///
  /// Derived from capture times rather than arrival times, so it reports the
  /// rate the system is actually running at rather than the rate this node
  /// managed to read the socket.
  double getCaptureRateHz() const
  {
    double const span = lastCaptureTime - firstCaptureTime;
    if (!hasData() || span <= 0.0)
    {
      return 0.0;
    }
    return (receivedFrames + droppedFrames - 1) / span;
  }

  /// \brief Rate at which frames were actually delivered to this node.
  double getReceivedRateHz() const
  {
    double const span = lastCaptureTime - firstCaptureTime;
    if (!hasData() || span <= 0.0)
    {
      return 0.0;
    }
    return (receivedFrames - 1) / span;
  }

  double getMeanMarkerError() const
  {
    if (receivedFrames <= 0)
    {
      return 0.0;
    }
    return markerErrorSum / receivedFrames;
  }

  double getMaxMarkerError() const
  {
    return markerErrorMax;
  }

private:
  bool hasAny;
  int firstFrameNumber;
  int lastFrameNumber;
  double firstCaptureTime;
  double lastCaptureTime;
  int receivedFrames;
  int droppedFrames;
  int untrackedFrames;
  int largestGap;
  double markerErrorSum;
  double markerErrorMax;
};

}  // namespace mocap_optitrack

#endif  // __MOCAP_OPTITRACK_FRAME_STATISTICS_H__
