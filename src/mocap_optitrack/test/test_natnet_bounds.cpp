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

#include "natnet/natnet_messages.h"

namespace
{

/// Build a well formed NatNet 3.0 data frame that can then be damaged.
natnet::MessageBuffer wellFormedFrame(int rigidBodies = 1)
{
  natnet::MessageBuffer b;
  auto put = [&b](void const* raw, size_t n)
  {
    char const* p = static_cast<char const*>(raw);
    b.insert(b.end(), p, p + n);
  };
  auto putI32 = [&](int32_t v) { put(&v, sizeof(v)); };
  auto putI16 = [&](int16_t v) { put(&v, sizeof(v)); };
  auto putU16 = [&](uint16_t v) { put(&v, sizeof(v)); };
  auto putU32 = [&](uint32_t v) { put(&v, sizeof(v)); };
  auto putU64 = [&](uint64_t v) { put(&v, sizeof(v)); };
  auto putF = [&](float v) { put(&v, sizeof(v)); };
  auto putD = [&](double v) { put(&v, sizeof(v)); };

  putU16(7);      // FrameOfData
  putU16(0);
  putI32(1);      // frame number
  putI32(0);      // marker sets
  putI32(0);      // unlabeled markers

  putI32(rigidBodies);
  for (int i = 0; i < rigidBodies; ++i)
  {
    putI32(i);
    putF(1.0f); putF(2.0f); putF(3.0f);
    putF(0.0f); putF(0.0f); putF(0.0f); putF(1.0f);
    putF(0.001f);
    putI16(0x01);
  }

  putI32(0);      // skeletons
  putI32(0);      // labeled markers
  putI32(0);      // force plates
  putI32(0);      // devices
  putU32(0);      // timecode
  putU32(0);      // timecode subframe
  putD(12.5);     // capture timestamp
  putU64(0); putU64(0); putU64(0);
  putI16(0);      // frame params
  putI32(0);      // end of data
  return b;
}

mocap_optitrack::DataModel makeModel()
{
  mocap_optitrack::DataModel model;
  int natNet[4] = {3, 0, 0, 0};
  int server[4] = {2, 0, 0, 0};
  model.setVersions(natNet, server);
  return model;
}

/// Overwrite a 32 bit field at a byte offset, to forge a count.
void pokeI32(natnet::MessageBuffer& b, size_t offset, int32_t value)
{
  ASSERT_LE(offset + sizeof(value), b.size());
  std::memcpy(&b[offset], &value, sizeof(value));
}

}  // namespace


// Baseline: the undamaged fixture parses, so failures below are attributable
// to the damage and not to a broken fixture.
TEST(NatNetBounds, WellFormedFrameStillParses)
{
  mocap_optitrack::DataModel model = makeModel();
  natnet::MessageDispatcher::dispatch(wellFormedFrame(), &model);

  ASSERT_EQ(1u, model.dataFrame.rigidBodies.size());
  EXPECT_TRUE(model.dataFrame.hasTimestamp);
  EXPECT_DOUBLE_EQ(12.5, model.dataFrame.timestamp);
}


// A datagram cut short mid-frame must be rejected rather than parsed from
// whatever bytes happen to follow in memory.
TEST(NatNetBounds, TruncatedFrameIsRejectedAtEveryLength)
{
  natnet::MessageBuffer full = wellFormedFrame();

  // Every truncation point, not just a convenient one.
  for (size_t len = 0; len < full.size(); ++len)
  {
    natnet::MessageBuffer truncated(full.begin(), full.begin() + len);
    mocap_optitrack::DataModel model = makeModel();

    natnet::MessageDispatcher::dispatch(truncated, &model);

    // The frame is incomplete, so no usable timestamp may be reported.
    EXPECT_FALSE(model.dataFrame.hasTimestamp)
      << "a frame truncated to " << len << " bytes was accepted";
  }
}


// An empty buffer must not be indexed.
TEST(NatNetBounds, EmptyBufferIsIgnored)
{
  mocap_optitrack::DataModel model = makeModel();
  natnet::MessageBuffer empty;

  natnet::MessageDispatcher::dispatch(empty, &model);

  EXPECT_TRUE(model.dataFrame.rigidBodies.empty());
  EXPECT_FALSE(model.dataFrame.hasTimestamp);
}


// A corrupted element count must not be trusted to size a container or drive a
// loop: the count lives in the packet, so it can claim billions of items.
TEST(NatNetBounds, AbsurdRigidBodyCountIsRejected)
{
  natnet::MessageBuffer frame = wellFormedFrame();
  // offset 4 frame number, 4 marker sets, 4 unlabeled, then the count
  pokeI32(frame, 4 + 4 + 4 + 4, 0x3FFFFFFF);

  mocap_optitrack::DataModel model = makeModel();
  natnet::MessageDispatcher::dispatch(frame, &model);

  EXPECT_TRUE(model.dataFrame.rigidBodies.empty());
  EXPECT_FALSE(model.dataFrame.hasTimestamp);
}


TEST(NatNetBounds, NegativeCountsAreRejected)
{
  natnet::MessageBuffer frame = wellFormedFrame();
  pokeI32(frame, 4 + 4, -1);   // marker set count

  mocap_optitrack::DataModel model = makeModel();
  natnet::MessageDispatcher::dispatch(frame, &model);

  EXPECT_FALSE(model.dataFrame.hasTimestamp);
}


TEST(NatNetBounds, AbsurdUnlabeledMarkerCountIsRejected)
{
  natnet::MessageBuffer frame = wellFormedFrame();
  pokeI32(frame, 4 + 4 + 4, 0x3FFFFFFF);

  mocap_optitrack::DataModel model = makeModel();
  natnet::MessageDispatcher::dispatch(frame, &model);

  EXPECT_TRUE(model.dataFrame.otherMarkers.empty());
  EXPECT_FALSE(model.dataFrame.hasTimestamp);
}


// A marker set name with no terminator must not be copied past the buffer, and
// must not overflow the fixed size destination.
TEST(NatNetBounds, UnterminatedMarkerSetNameIsRejected)
{
  natnet::MessageBuffer b;
  auto putI32 = [&b](int32_t v)
  {
    char const* p = reinterpret_cast<char const*>(&v);
    b.insert(b.end(), p, p + sizeof(v));
  };
  uint16_t id = 7, len = 0;
  b.insert(b.end(), reinterpret_cast<char*>(&id), reinterpret_cast<char*>(&id) + 2);
  b.insert(b.end(), reinterpret_cast<char*>(&len), reinterpret_cast<char*>(&len) + 2);
  putI32(1);   // frame number
  putI32(1);   // one marker set
  // Name bytes that never terminate, and run to the end of the buffer.
  for (int i = 0; i < 64; ++i)
  {
    b.push_back('A');
  }

  mocap_optitrack::DataModel model = makeModel();
  natnet::MessageDispatcher::dispatch(b, &model);

  EXPECT_FALSE(model.dataFrame.hasTimestamp);
}


// Version mismatch is the realistic way a well formed packet disagrees with the
// layout being applied: a 3.0 frame read as 2.0 runs off the end.
TEST(NatNetBounds, VersionMismatchIsRejectedRatherThanMisparsed)
{
  natnet::MessageBuffer frame = wellFormedFrame(64);

  mocap_optitrack::DataModel model;
  int natNet[4] = {2, 0, 0, 0};   // deliberately wrong
  int server[4] = {2, 0, 0, 0};
  model.setVersions(natNet, server);

  natnet::MessageDispatcher::dispatch(frame, &model);

  EXPECT_FALSE(model.dataFrame.hasTimestamp);
}


// A short server info message must not be read as a full sender block.
TEST(NatNetBounds, ShortServerInfoIsIgnored)
{
  mocap_optitrack::DataModel model;
  natnet::MessageBuffer b;
  uint16_t id = 1, len = 0;      // MessageType::ServerInfo
  b.insert(b.end(), reinterpret_cast<char*>(&id), reinterpret_cast<char*>(&id) + 2);
  b.insert(b.end(), reinterpret_cast<char*>(&len), reinterpret_cast<char*>(&len) + 2);
  b.resize(32);                  // far short of a sender block

  natnet::MessageDispatcher::dispatch(b, &model);

  EXPECT_FALSE(model.hasServerInfo());
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  // The deserializer logs through throttled macros, which read the ROS clock.
  // Nothing here runs a node, so the clock has to be started by hand.
  ros::Time::init();
  return RUN_ALL_TESTS();
}
