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

// Local includes
#include <mocap_optitrack/socket.h>
#include <mocap_optitrack/data_model.h>
#include <mocap_optitrack/mocap_config.h>
#include <mocap_optitrack/rigid_body_publisher.h>
#include <mocap_optitrack/timestamp_sync.h>
#include <mocap_optitrack/frame_statistics.h>
#include <mocap_optitrack/MocapOptitrackConfig.h>
#include "natnet/natnet_messages.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <memory>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>


namespace mocap_optitrack
{

class OptiTrackRosBridge
{
public:
  OptiTrackRosBridge(ros::NodeHandle& nh,
                     ServerDescription const& serverDescr,
                     PublisherConfigurations const& pubConfigs) :
    nh(nh), server(ros::NodeHandle("~/optitrack_config")),
    initialized(false), useMocapTimestamps(true), measuredRatePublished(false)
  {
    server.setCallback(boost::bind(&OptiTrackRosBridge::reconfigureCallback, this, _1, _2));
    serverDescription = serverDescr;
    publisherConfigurations = pubConfigs;

    // Stamping against Motive's capture time is the correct behaviour, but it
    // is left switchable: a server that reports no timestamp, or a deployment
    // that deliberately wants arrival time, can fall back without a rebuild.
    ros::NodeHandle privateNh("~");
    privateNh.param("use_mocap_timestamps", useMocapTimestamps, true);

    // The frame counter is the only way a consumer can tell a dropped frame
    // from a merely delayed one, so it is published alongside the poses.
    frameNumberPublisher = nh.advertise<std_msgs::UInt32>("frame_number", 1000);

    // Stream health is otherwise only apparent after the fact, from a recording.
    // Reporting it live lets a degraded run be noticed while it is still running.
    diagnosticUpdater.setHardwareID("optitrack");
    diagnosticUpdater.add("Mocap stream", this, &OptiTrackRosBridge::produceDiagnostics);

    // Latched, so a recording started at any point still captures it. Nothing
    // else in the stream records what produced it, which leaves later analysis
    // guessing at the capture rate and protocol version it was taken with.
    serverInfoPublisher = nh.advertise<std_msgs::String>("server_info", 1, true);
  }

  void reconfigureCallback(MocapOptitrackConfig& config, uint32_t)
  {
    serverDescription.enableOptitrack = config.enable_optitrack;
    serverDescription.commandPort = config.command_port;
    serverDescription.dataPort = config.data_port;
    serverDescription.multicastIpAddress = config.multicast_address;

    initialize();
  }

  void initialize()
  {
    if (serverDescription.enableOptitrack)
    {
      // Create socket
      multicastClientSocketPtr.reset(
        new UdpMulticastSocket(serverDescription.dataPort,
                               serverDescription.multicastIpAddress));

      if (!serverDescription.version.empty())
      {
        dataModel.setVersions(&serverDescription.version[0], &serverDescription.version[0]);
      }

      // Need verion information from the server to properly decode any of their packets.
      // If we have not recieved that yet, send another request.
      while (ros::ok() && !dataModel.hasServerInfo())
      {
        natnet::ConnectionRequestMessage connectionRequestMsg;
        natnet::MessageBuffer connectionRequestMsgBuffer;
        connectionRequestMsg.serialize(connectionRequestMsgBuffer, NULL);
        int ret = multicastClientSocketPtr->send(
                    &connectionRequestMsgBuffer[0],
                    connectionRequestMsgBuffer.size(),
                    serverDescription.commandPort);
        if (updateDataModelFromServer()) usleep(10);
        else sleep(1);

        ros::spinOnce();
      }
      // Once we have the server info, create publishers
      publishDispatcherPtr.reset(
        new RigidBodyPublishDispatcher(nh,
                                       dataModel.getNatNetVersion(),
                                       publisherConfigurations));
      ROS_INFO("Initialization complete");
      initialized = true;

      // Publish what is known now; republished once with the measured capture
      // rate after enough frames have been seen to estimate it.
      provenanceStatistics.reset();
      measuredRatePublished = false;
      publishServerInfo();
    }
    else
    {
      ROS_INFO("Initialization incomplete");
      initialized = false;
    }
  };

  void run()
  {
    while (ros::ok())
    {
      if (initialized)
      {
        if (updateDataModelFromServer())
        {
          // Maybe we got some data? If we did it would be in the form of one or more
          // rigid bodies in the data model
          ros::Time time = resolveFrameTime(dataModel.dataFrame);
          publishDispatcherPtr->publish(time, dataModel.dataFrame.rigidBodies);

          std_msgs::UInt32 frameNumberMsg;
          frameNumberMsg.data = static_cast<uint32_t>(dataModel.frameNumber);
          frameNumberPublisher.publish(frameNumberMsg);

          accumulateStatistics(time);

          // Clear out the model to prepare for the next frame of data
          dataModel.clear();
        }
        // whether receive or nor, give a short break to relieft the CPU load due to while()
        usleep(100);
      }
      else
      {
        ros::Duration(1.).sleep();
      }
      // Outside the branch above: a node that never connected is exactly the
      // case a consumer most needs told about, so diagnostics have to keep
      // being published when there is no data rather than fall silent.
      diagnosticUpdater.update();
      ros::spinOnce();
    }
  }

private:
  /// \brief Decide what time to stamp this frame with.
  ///
  /// Arrival time is only a stand-in for capture time when the socket is
  /// drained promptly. It is not: the receive loop polls, so a queued backlog
  /// is emitted in a burst and successive frames land far closer together than
  /// the 1/framerate they were actually captured at. Motive's own timestamp is
  /// immune to that, so it is preferred whenever the server supplies one.
  ros::Time resolveFrameTime(ModelFrame const& frame)
  {
    ros::Time const now = ros::Time::now();

    if (!useMocapTimestamps || !frame.hasTimestamp)
    {
      ROS_WARN_ONCE_NAMED("timestamps",
        "Stamping mocap poses with arrival time. Intervals between poses will "
        "not reflect true capture intervals; differentiating them for velocity "
        "or acceleration will be inaccurate.");
      return now;
    }

    ros::Time const stamp(timestampSync.toRosTime(frame.timestamp, now.toSec()));

    ROS_INFO_ONCE_NAMED("timestamps",
      "Stamping mocap poses with Motive capture time (offset %.6f s).",
      timestampSync.getOffset());

    return stamp;
  }

  /// \brief Describe what produced this stream, as a YAML mapping.
  ///
  /// A recording of poses alone does not say what protocol version decoded it,
  /// what rate the system was capturing at, or which clock stamped it, yet all
  /// three are needed to interpret the data later. Publishing them latched puts
  /// them in the recording alongside the poses.
  void publishServerInfo()
  {
    std::ostringstream info;
    info << "natnet_version: \"" << dataModel.getNatNetVersion().getVersionString() << "\"\n"
         << "server_version: \"" << dataModel.getServerVersion().getVersionString() << "\"\n"
         << "timestamp_source: \""
         << (useMocapTimestamps ? "mocap_capture_time" : "message_arrival_time") << "\"\n"
         << "multicast_address: \"" << serverDescription.multicastIpAddress << "\"\n"
         << "data_port: " << serverDescription.dataPort << "\n"
         << "command_port: " << serverDescription.commandPort << "\n";

    // Only meaningful once enough frames have been seen to measure it.
    if (provenanceStatistics.hasData())
    {
      info << "measured_capture_rate_hz: "
           << provenanceStatistics.getCaptureRateHz() << "\n";
    }

    info << "rigid_body_ids: [";
    for (size_t i = 0; i < publisherConfigurations.size(); ++i)
    {
      info << (i ? ", " : "") << publisherConfigurations[i].rigidBodyId;
    }
    info << "]\n";

    std_msgs::String msg;
    msg.data = info.str();
    serverInfoPublisher.publish(msg);
  }

  /// \brief Fold the frame just received into the running stream statistics.
  void accumulateStatistics(ros::Time const& frameTime)
  {
    bool anyTracked = false;
    double markerErrorSum = 0.0;
    for (auto const& body : dataModel.dataFrame.rigidBodies)
    {
      anyTracked = anyTracked || body.isTrackingValid;
      markerErrorSum += body.meanMarkerError;
    }
    double const meanMarkerError = dataModel.dataFrame.rigidBodies.empty()
      ? 0.0
      : markerErrorSum / dataModel.dataFrame.rigidBodies.size();

    frameStatistics.update(dataModel.frameNumber, frameTime.toSec(),
                           anyTracked, meanMarkerError);

    // A second, never reset accumulator, purely to measure the capture rate
    // for the provenance message. Republished once when the estimate has had
    // enough frames to settle; latched, so the last one is what a recording
    // picks up.
    if (!measuredRatePublished)
    {
      provenanceStatistics.update(dataModel.frameNumber, frameTime.toSec(),
                                  anyTracked, meanMarkerError);
      if (provenanceStatistics.getReceivedFrames() >= kRateEstimateFrames)
      {
        publishServerInfo();
        measuredRatePublished = true;
      }
    }
  }

  /// \brief Report stream health, and reset the window so each report covers
  ///        the interval since the last one rather than the whole session.
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (!initialized)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Not connected to a mocap server");
      return;
    }

    if (!frameStatistics.hasData())
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Connected but receiving no frames");
      frameStatistics.reset();
      return;
    }

    double const dropPercent = frameStatistics.getDropPercent();
    double const untrackedPercent = frameStatistics.getUntrackedPercent();

    if (dropPercent > 5.0)
    {
      status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Losing %.1f%% of frames", dropPercent);
    }
    else if (dropPercent > 1.0)
    {
      status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                      "Losing %.1f%% of frames", dropPercent);
    }
    else if (untrackedPercent > 50.0)
    {
      status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                      "No body solved in %.1f%% of frames", untrackedPercent);
    }
    else
    {
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
                      "Streaming at %.1f Hz", frameStatistics.getCaptureRateHz());
    }

    status.add("Capture rate (Hz)", frameStatistics.getCaptureRateHz());
    status.add("Received rate (Hz)", frameStatistics.getReceivedRateHz());
    status.add("Frames received", frameStatistics.getReceivedFrames());
    status.add("Frames dropped", frameStatistics.getDroppedFrames());
    status.add("Frames dropped (%)", dropPercent);
    status.add("Largest consecutive gap", frameStatistics.getLargestGap());
    status.add("Frames with no body solved (%)", untrackedPercent);
    status.add("Mean marker error", frameStatistics.getMeanMarkerError());
    status.add("Max marker error", frameStatistics.getMaxMarkerError());
    status.add("Timestamp source",
               useMocapTimestamps ? "mocap capture time" : "arrival time");

    frameStatistics.reset();
  }

  bool updateDataModelFromServer()
  {
    // Get data from mocap server
    int numBytesReceived = multicastClientSocketPtr->recv();
    if (numBytesReceived > 0)
    {
      // Grab latest message buffer
      const char* pMsgBuffer = multicastClientSocketPtr->getBuffer();

      // Copy char* buffer into MessageBuffer and dispatch to be deserialized
      natnet::MessageBuffer msgBuffer(pMsgBuffer, pMsgBuffer + numBytesReceived);
      natnet::MessageDispatcher::dispatch(msgBuffer, &dataModel);

      return true;
    }

    return false;
  };

  ros::NodeHandle& nh;
  ServerDescription serverDescription;
  PublisherConfigurations publisherConfigurations;
  DataModel dataModel;
  std::unique_ptr<UdpMulticastSocket> multicastClientSocketPtr;
  std::unique_ptr<RigidBodyPublishDispatcher> publishDispatcherPtr;
  dynamic_reconfigure::Server<MocapOptitrackConfig> server;
  bool initialized;
  bool useMocapTimestamps;
  TimestampSynchronizer timestampSync;
  ros::Publisher frameNumberPublisher;
  FrameStatistics frameStatistics;
  diagnostic_updater::Updater diagnosticUpdater;
  ros::Publisher serverInfoPublisher;
  FrameStatistics provenanceStatistics;
  bool measuredRatePublished;

  /// Frames averaged before the measured capture rate is considered settled.
  static int const kRateEstimateFrames = 600;
};

}  // namespace mocap_optitrack


////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  // Initialize ROS node
  ros::init(argc, argv, "mocap_node");
  ros::NodeHandle nh("~");

  // Grab node configuration from rosparam
  mocap_optitrack::ServerDescription serverDescription;
  mocap_optitrack::PublisherConfigurations publisherConfigurations;
  mocap_optitrack::NodeConfiguration::fromRosParam(nh, serverDescription, publisherConfigurations);

  // Create node object, initialize and run
  mocap_optitrack::OptiTrackRosBridge node(nh, serverDescription, publisherConfigurations);
  node.initialize();
  node.run();

  return 0;
}
