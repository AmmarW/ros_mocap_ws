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

#include <cstdint>
#include <cstring>
#include <vector>

#include <ros/time.h>

#include <mocap_optitrack/data_model.h>
#include <mocap_optitrack/timestamp_sync.h>

#include "natnet/natnet_messages.h"

namespace
{

/// Assembles a NatNet FrameOfData packet byte-for-byte, so the production
/// deserializer is exercised on real wire format rather than on a stub.
class FrameBuilder
{
public:
  FrameBuilder()
  {
    append<uint16_t>(7);   // MessageType::FrameOfData
    append<uint16_t>(0);   // numDataBytes, unread by the deserializer
  }

  template <typename T>
  void append(T value)
  {
    char const* raw = reinterpret_cast<char const*>(&value);
    buffer.insert(buffer.end(), raw, raw + sizeof(T));
  }

  /// Build a single-rigid-body frame for NatNet 3.0, the version the recorded
  /// rig streams. Section order follows the NatNet 3.0 data frame layout.
  static natnet::MessageBuffer buildV3(int frameNumber, double timestamp)
  {
    FrameBuilder b;
    b.append<int32_t>(frameNumber);
    b.append<int32_t>(0);            // marker sets
    b.append<int32_t>(0);            // unlabeled markers

    b.append<int32_t>(1);            // rigid bodies
    b.append<int32_t>(7);            //   body id (matches config/mocap.yaml)
    b.append<float>(1.0f);           //   position x, y, z
    b.append<float>(2.0f);
    b.append<float>(3.0f);
    b.append<float>(0.0f);           //   orientation x, y, z, w
    b.append<float>(0.0f);
    b.append<float>(0.0f);
    b.append<float>(1.0f);
    b.append<float>(0.001f);         //   mean marker error   (>= 2.0)
    b.append<int16_t>(0x01);         //   params, tracking valid (>= 2.6)

    b.append<int32_t>(0);            // skeletons        (>= 2.1)
    b.append<int32_t>(0);            // labeled markers  (>= 2.3)
    b.append<int32_t>(0);            // force plates     (>= 2.9)
    b.append<int32_t>(0);            // devices          (>= 3.0)
    // software latency is absent from 3.0 onwards

    b.append<uint32_t>(0);           // timecode
    b.append<uint32_t>(0);           // timecode subframe
    b.append<double>(timestamp);     // capture instant  (>= 2.7)

    b.append<uint64_t>(0);           // camera mid-exposure (>= 3.0)
    b.append<uint64_t>(0);           // camera data received
    b.append<uint64_t>(0);           // transmit

    b.append<int16_t>(0);            // frame params
    b.append<int32_t>(0);            // end of data
    return b.buffer;
  }

  natnet::MessageBuffer buffer;
};

mocap_optitrack::DataModel makeModel()
{
  mocap_optitrack::DataModel model;
  int natNet[4] = {3, 0, 0, 0};
  int server[4] = {2, 0, 0, 0};
  model.setVersions(natNet, server);
  return model;
}

}  // namespace


// The capture instant is on the wire in every frame; it must reach the model
// rather than being parsed and dropped.
TEST(NatNetTimestamp, CaptureTimestampReachesTheDataModel)
{
  mocap_optitrack::DataModel model = makeModel();
  natnet::MessageBuffer packet = FrameBuilder::buildV3(4242, 1234.5678);

  natnet::MessageDispatcher::dispatch(packet, &model);

  EXPECT_EQ(4242, model.frameNumber);
  ASSERT_TRUE(model.dataFrame.hasTimestamp);
  EXPECT_DOUBLE_EQ(1234.5678, model.dataFrame.timestamp);

  // and the frame itself must still parse correctly
  ASSERT_EQ(1u, model.dataFrame.rigidBodies.size());
  EXPECT_EQ(7, model.dataFrame.rigidBodies[0].bodyId);
  EXPECT_TRUE(model.dataFrame.rigidBodies[0].hasValidData());
  EXPECT_FLOAT_EQ(1.0f, model.dataFrame.rigidBodies[0].pose.position.x);
  EXPECT_FLOAT_EQ(3.0f, model.dataFrame.rigidBodies[0].pose.position.z);
}


// Motive counts seconds from its own startup, so the earliest frames carry
// timestamps at or near zero. Those are real captures and must be honoured;
// treating zero as "missing" would discard the start of every recording.
TEST(NatNetTimestamp, ZeroTimestampIsValidBecauseMotiveCountsFromStartup)
{
  mocap_optitrack::DataModel model = makeModel();

  natnet::MessageDispatcher::dispatch(FrameBuilder::buildV3(1, 0.0), &model);

  EXPECT_TRUE(model.dataFrame.hasTimestamp);
  EXPECT_DOUBLE_EQ(0.0, model.dataFrame.timestamp);
}


// A nonsensical timestamp must be detectable, so the node can fall back to
// arrival time rather than stamping poses before the epoch.
TEST(NatNetTimestamp, NegativeTimestampIsFlaggedAsUnusable)
{
  mocap_optitrack::DataModel model = makeModel();

  natnet::MessageDispatcher::dispatch(FrameBuilder::buildV3(1, -1.0), &model);

  EXPECT_FALSE(model.dataFrame.hasTimestamp);
}


// clear() must not leave a stale timestamp behind for the next frame.
TEST(NatNetTimestamp, ClearResetsTimestampState)
{
  mocap_optitrack::DataModel model = makeModel();
  natnet::MessageDispatcher::dispatch(FrameBuilder::buildV3(1, 99.5), &model);
  ASSERT_TRUE(model.dataFrame.hasTimestamp);

  model.clear();

  EXPECT_FALSE(model.dataFrame.hasTimestamp);
  EXPECT_DOUBLE_EQ(0.0, model.dataFrame.timestamp);
}


// End to end over the parsing path: frames captured at a uniform rate but
// delivered in a burst must still be published at the uniform rate. This is the
// regression the driver actually had.
TEST(NatNetTimestamp, BurstyDeliveryStillYieldsUniformPublishedIntervals)
{
  double const framePeriod = 1.0 / 120.0;
  double const epoch = 1762822941.0;
  int const calibration = 240;

  mocap_optitrack::DataModel model = makeModel();
  mocap_optitrack::TimestampSynchronizer sync(calibration);

  std::vector<double> stamps;
  for (int i = 0; i < 1200; ++i)
  {
    double const capture = i * framePeriod;

    // Stall for 300 ms every 200 frames, then drain the backlog back-to-back.
    int const phase = i % 200;
    double const delay =
      (i > 200 && phase < 36) ? 0.300 - phase * framePeriod : 0.0002;

    natnet::MessageDispatcher::dispatch(
      FrameBuilder::buildV3(i, capture), &model);
    ASSERT_TRUE(model.dataFrame.hasTimestamp) << "frame " << i;

    stamps.push_back(
      sync.toRosTime(model.dataFrame.timestamp, epoch + capture + delay));
    model.clear();
  }

  for (size_t i = calibration + 1; i < stamps.size(); ++i)
  {
    EXPECT_NEAR(framePeriod, stamps[i] - stamps[i - 1], 1e-6)
      << "published interval " << i << " was distorted by delivery timing";
  }
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  // The deserializer logs through throttled macros, which read the ROS clock.
  // Nothing here runs a node, so the clock has to be started by hand.
  ros::Time::init();
  return RUN_ALL_TESTS();
}
