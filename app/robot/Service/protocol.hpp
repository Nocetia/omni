#pragma once

#include "crc.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace robot
{

    namespace vision
    {
        inline constexpr std::size_t vision_command_size = 28;
        inline constexpr std::size_t vision_telemetry_size = 44;

        static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
            "The legacy wire protocol requires IEEE 754 binary32 floats");

        struct vision_command
        {
            uint8_t mode{};
            // Angles in rad, velocities in rad/s, accelerations in rad/s^2.
            float yaw{};
            float yaw_vel{};
            float yaw_acc{};
            float pitch{};
            float pitch_vel{};
            float pitch_acc{};
        };

        struct vision_telemetry
        {
            bool detect_color{};
            bool reset_tracker{};
            uint8_t mode{};
            // W, X, Y, Z; body-to-world in the agreed gimbal frame.
            float q[4]{1.0f, 0.0f, 0.0f, 0.0f};
            float yaw{};                        // Degrees; continuous yaw.
            float gyro_yaw{};                   // Body Z, rad/s.
            float pitch{};                      // Degrees.
            float gyro_pitch{};                 // Body Y, rad/s.
            float bullet_speed{};               // m/s.
            uint16_t bullet_count{};
        };


        inline uint32_t read_u32_le(const uint8_t* data)
        {
            return static_cast<uint32_t>(data[0]) | 
                (static_cast<uint32_t>(data[1]) << 8U) |
                (static_cast<uint32_t>(data[2]) << 16U) | 
                (static_cast<uint32_t>(data[3]) << 24U);
        }

        inline float read_float_le(const uint8_t* data)
        {
            const uint32_t bits = read_u32_le(data);
            float value;
            std::memcpy(&value, &bits, sizeof(value));
            return value;
        }

        inline bool finite_float_le(const uint8_t* data)
        {
            // Inspect the wire representation so validation also survives fast-math compiler options.
            return (read_u32_le(data) & 0x7F800000U) != 0x7F800000U;
        }

        inline void write_float_le(uint8_t* data, float value)
        {
            uint32_t bits;
            std::memcpy(&bits, &value, sizeof(bits));
            for (unsigned i = 0; i < sizeof(bits); ++i)
            {
                data[i] = static_cast<uint8_t>(bits >> (i * 8U));
            }
        }

        // Own one parser in the USB RX thread. Chunk boundaries do not affect frame detection.
        class vision_parser
        {
        public:
            bool push(uint8_t byte, vision_command& command)
            {
                if (size_ == 0 && byte != 0xA5)
                {
                    return false;
                }

                frame_[size_++] = byte;
                if (size_ != frame_.size())
                {
                    return false;
                }

                bool valid = crc::verify_crc16_checksum(frame_.data(), vision_command_size) != 0;
                for (std::size_t offset = 2; valid && offset < frame_.size() - 2; offset += 4)
                {
                    valid = finite_float_le(frame_.data() + offset);
                }
                if (valid)
                {
                    command = {frame_[1], 
                    read_float_le(frame_.data() + 2),
                    read_float_le(frame_.data() + 6), 
                    read_float_le(frame_.data() + 10),
                    read_float_le(frame_.data() + 14), 
                    read_float_le(frame_.data() + 18),
                    read_float_le(frame_.data() + 22)};
                    reset();
                    return true;
                }

                // A damaged frame may already contain the beginning of the next valid frame.
                std::size_t next = 1;
                while (next < size_ && frame_[next] != 0xA5)
                {
                    ++next;
                }
                size_ -= next;
                std::memmove(frame_.data(), frame_.data() + next, size_);
                return false;
            }

            void reset()
            {
                size_ = 0;
            }

        private:
            std::array<uint8_t, vision_command_size> frame_{};
            std::size_t size_{};
        };

        inline std::array<uint8_t, vision_telemetry_size> encode_vision(const vision_telemetry& value)
        {
            std::array<uint8_t, vision_telemetry_size> bytes{};
            bytes[0] = 0x5A;
            bytes[1] = static_cast<uint8_t>(value.detect_color | (value.reset_tracker << 1U));
            bytes[2] = value.mode;
            bytes[3] = 0; // reserved: uint8_t reserved : 2 in the legacy layout.
            for (unsigned i = 0; i < 4; ++i)
            {
                write_float_le(bytes.data() + 4 + i * 4, value.q[i]);
            }
            write_float_le(bytes.data() + 20, value.yaw);
            write_float_le(bytes.data() + 24, value.gyro_yaw);
            write_float_le(bytes.data() + 28, value.pitch);
            write_float_le(bytes.data() + 32, value.gyro_pitch);
            write_float_le(bytes.data() + 36, value.bullet_speed);
            bytes[40] = static_cast<uint8_t>(value.bullet_count);
            bytes[41] = static_cast<uint8_t>(value.bullet_count >> 8U);
            crc::append_crc16_checksum(bytes.data(), vision_telemetry_size);
            return bytes;
        }

    }



} // namespace robot
