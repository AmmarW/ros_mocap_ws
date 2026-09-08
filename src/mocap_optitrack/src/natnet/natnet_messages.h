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
#ifndef __MOCAP_OPTITRACK_NATNET_MESSAGES_H__
#define __MOCAP_OPTITRACK_NATNET_MESSAGES_H__

#include <cstddef>
#include <cstring>
#include <vector>
#include <mocap_optitrack/data_model.h>

namespace natnet
{
    typedef std::vector<char> MessageBuffer;

    /// \brief Bounds checked cursor over a received message.
    ///
    /// Packet contents are attacker- or fault-controlled in the sense that a
    /// truncated datagram, a version mismatch or a corrupted field can all make
    /// the declared structure disagree with the bytes actually present. Reading
    /// through a bare iterator walks off the end of the buffer when that
    /// happens. This cursor refuses to read past the end, latches the fact that
    /// it happened, and turns every subsequent read into a no-op so a caller can
    /// parse straight through and check once at the end.
    class BufferReader
    {
    public:
        explicit BufferReader(MessageBuffer const& buffer)
          : iter(buffer.begin()), end(buffer.end()), overrun(false)
        {
        }

        /// \brief Copy one value out of the buffer and advance past it.
        /// \return False if there were not enough bytes, leaving target alone.
        template <typename T>
        bool read(T& target)
        {
            if (overrun || remaining() < sizeof(T))
            {
                overrun = true;
                return false;
            }
            std::memcpy(&target, &(*iter), sizeof(T));
            iter += sizeof(T);
            return true;
        }

        /// \brief Advance without reading.
        bool skip(std::size_t bytes)
        {
            if (overrun || remaining() < bytes)
            {
                overrun = true;
                return false;
            }
            iter += bytes;
            return true;
        }

        /// \brief Copy a null terminated string, which the wire format uses for
        ///        names, refusing to run past the end if the terminator is absent.
        bool readString(char* dest, std::size_t destSize)
        {
            if (overrun || destSize == 0)
            {
                overrun = true;
                return false;
            }

            std::size_t length = 0;
            while (length < remaining() && iter[length] != '\0')
            {
                ++length;
            }

            // No terminator inside the buffer, or no room for the copy.
            if (length >= remaining() || length + 1 > destSize)
            {
                overrun = true;
                return false;
            }

            std::memcpy(dest, &(*iter), length + 1);
            iter += length + 1;
            return true;
        }

        /// \brief False once any read has run past the end of the buffer.
        bool ok() const
        {
            return !overrun;
        }

        std::size_t remaining() const
        {
            return overrun ? 0u : static_cast<std::size_t>(end - iter);
        }

        /// \brief Whether count items of itemSize bytes could still fit.
        ///
        /// Element counts are read from the packet itself, so a corrupted one
        /// can ask for billions of items. Checking before looping keeps a bad
        /// count from turning into a very long loop over failing reads.
        bool canHold(int count, std::size_t itemSize) const
        {
            if (count < 0)
            {
                return false;
            }
            return static_cast<std::size_t>(count) <= remaining() / (itemSize ? itemSize : 1);
        }

    private:
        MessageBuffer::const_iterator iter;
        MessageBuffer::const_iterator end;
        bool overrun;
    };

    struct MessageInterface
    {
        virtual void serialize(MessageBuffer&, mocap_optitrack::DataModel const*) {};
        virtual void deserialize(MessageBuffer const&, mocap_optitrack::DataModel*) {};
    };

    struct ConnectionRequestMessage : public MessageInterface
    {
        virtual void serialize(MessageBuffer& msgBuffer, mocap_optitrack::DataModel const*);
    };

    struct ServerInfoMessage : public MessageInterface
    {
        virtual void deserialize(MessageBuffer const&, mocap_optitrack::DataModel*);
    };

    class DataFrameMessage : public MessageInterface
    {
        struct RigidBodyMessagePart
        {
            void deserialize(BufferReader&,
                mocap_optitrack::RigidBody&,
                mocap_optitrack::Version const&);
        };

    public:
        virtual void deserialize(MessageBuffer const&, mocap_optitrack::DataModel*);
    };

    struct MessageDispatcher
    {
        /// \brief Decode one received message into the model.
        /// \return True only when the message was a data frame that decoded
        ///         successfully. A caller must not treat anything else, such as
        ///         the server info reply, as a frame of pose data.
        static bool dispatch(MessageBuffer const&, mocap_optitrack::DataModel*);
    };
}

#endif