/*
 * Copyright (c) 2011 University of Bonn, Computer Science Institute,
 * Kathrin Gräve
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
#include "mocap_optitrack/socket.h"
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>
#include <ros/ros.h>

UdpMulticastSocket::UdpMulticastSocket(const int local_port,
                                       const std::string multicast_ip,
                                       const std::string local_interface_ip)
{
  remote_ip_exist = false;
  memset(&HostAddr, 0, sizeof(HostAddr));

  // Create a UDP socket
  ROS_INFO("Creating socket...");
  m_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (m_socket < 0)
    throw SocketException(strerror(errno));

  // Allow reuse of local addresses
  ROS_INFO("Setting socket options...");
  int option_value = 1;
  int result = setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, (void*)&option_value, sizeof(int));
  if (result == -1)
  {
    std::stringstream error;
    error << "Failed to set socket option: ";
    switch (errno)
    {
    case EBADF:
      error << "EBADF";
      break;
    case EFAULT:
      error << "EFAULT";
      break;
    case EINVAL:
      error << "EINVAL";
      break;
    case ENOPROTOOPT:
      error << "ENOPROTOOPT";
      break;
    case ENOTSOCK:
      error << "ENOTSOCK";
      break;
    default:
      error << "unknown error";
      break;
    }
    throw SocketException(error.str().c_str());
  }

  // Fill struct for local address
  memset(&m_local_addr, 0, sizeof(m_local_addr));
  m_local_addr.sin_family = AF_INET;
  m_local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  m_local_addr.sin_port = htons(local_port);
  ROS_INFO("Local address: %s:%i", inet_ntoa(m_local_addr.sin_addr), ntohs(m_local_addr.sin_port));

  // Bind the socket
  ROS_INFO("Binding socket to local address...");
  result = bind(m_socket, (sockaddr*)&m_local_addr, sizeof(m_local_addr));
  if (result == -1)
  {
    std::stringstream error;
    error << "Failed to bind socket to local address:" << strerror(errno);
    throw SocketException(error.str().c_str());
  }

  // Join multicast group
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip.c_str());

  // Choosing the interface explicitly matters on a multi-homed host. Left to
  // the kernel, the group is joined on whichever interface the route to the
  // group selects, which is normally the one holding the default route rather
  // than the one carrying the mocap traffic. The stream then never arrives,
  // with nothing to indicate why.
  if (local_interface_ip.empty())
  {
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    ROS_INFO("Joining multicast group %s on the interface chosen by the route "
             "to it. Set multicast_interface to pin it to one interface.",
             multicast_ip.c_str());
  }
  else
  {
    mreq.imr_interface.s_addr = inet_addr(local_interface_ip.c_str());
    if (mreq.imr_interface.s_addr == INADDR_NONE)
    {
      std::stringstream error;
      error << "multicast_interface '" << local_interface_ip
            << "' is not a valid address of a local interface";
      throw SocketException(error.str().c_str());
    }
    ROS_INFO("Joining multicast group %s on interface %s...",
             multicast_ip.c_str(), local_interface_ip.c_str());
  }

  result = setsockopt(m_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq));
  if (result == -1)
  {
    std::stringstream error;
    error << "Failed to set socket option: ";
    switch (errno)
    {
    case EBADF:
      error << "EBADF";
      break;
    case EFAULT:
      error << "EFAULT";
      break;
    case EINVAL:
      error << "EINVAL";
      break;
    case ENOPROTOOPT:
      error << "ENOPROTOOPT";
      break;
    case ENOTSOCK:
      error << "ENOTSOCK";
      break;
    default:
      error << "unknown error";
      break;
    }
    throw SocketException(error.str().c_str());
  }

  // Make socket non-blocking
  ROS_INFO("Enabling non-blocking I/O");
  int flags = fcntl(m_socket, F_GETFL, 0);
  result = fcntl(m_socket, F_SETFL, flags | O_NONBLOCK);
  if (result == -1)
  {
    std::stringstream error;
    error << "Failed to enable non-blocking I/O: " << strerror(errno);
    throw SocketException(error.str().c_str());
  }
}

UdpMulticastSocket::~UdpMulticastSocket()
{
  close(m_socket);
}

int UdpMulticastSocket::recv()
{
  memset(buf, 0, MAXRECV + 1);

  sockaddr_in remote_addr;
  int addr_len = sizeof(struct sockaddr);
  // MSG_TRUNC makes the return value the true datagram length rather than the
  // number of bytes copied, which is the only way to notice that a datagram was
  // larger than the buffer. Without it an oversized frame is delivered with its
  // tail silently removed and looks like a valid short frame.
  int status = recvfrom(
                 m_socket,
                 buf,
                 MAXRECV,
                 MSG_TRUNC,
                 (sockaddr *)&remote_addr,
                 (socklen_t*)&addr_len);

  if (status > MAXRECV)
  {
    ROS_WARN_THROTTLE(5.0,
      "Received a %i byte datagram but the buffer holds %i; the frame was "
      "truncated and is being discarded.", status, MAXRECV);
    return -1;
  }

  if (status > 0)
  {
    ROS_DEBUG("%4i bytes received from %s:%i", status, inet_ntoa(remote_addr.sin_addr), ntohs(remote_addr.sin_port));

    // Only a successful receive leaves remote_addr meaningful. recvfrom does
    // not touch it when it fails, and on a non-blocking socket failing is the
    // common case, so recording it unconditionally would overwrite the peer
    // address with whatever was on the stack. Requests addressed to the server
    // would then be sent to a junk address, and would do so far more often than
    // not, since most receive attempts return nothing.
    HostAddr.sin_addr = remote_addr.sin_addr;
    remote_ip_exist = true;
  }
  else if (status == 0)
  {
    ROS_DEBUG("Connection closed by peer");
  }

  return status;
}

int UdpMulticastSocket::send(const char* buf, unsigned int sz, int port)
{
  // The server's address is learned from the frames it streams, so there is
  // nothing to address until one has arrived. Sending before then would go to
  // an address that was never set.
  if (!remote_ip_exist)
  {
    return -1;
  }

  HostAddr.sin_family = AF_INET;
  HostAddr.sin_port = htons(port);
  return sendto(m_socket, buf, sz, 0, (sockaddr*)&HostAddr, sizeof(HostAddr));
}
