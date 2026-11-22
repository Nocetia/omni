#include "Service/protocol.hpp"

#include <array>
#include <cstdio>
#include <vector>

namespace
{
using namespace robot::vision;

unsigned checks = 0;
unsigned failures = 0;

// Handwritten wire values: mode 7; yaw/rate/accel 0.5, -2, 3;
// pitch/rate/accel -4, 5, -6. CRC bytes are fixed independently of the production encoder.
constexpr std::array<uint8_t, 28> command_frame = {0xA5, 0x07, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00,
    0x00, 0xC0, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80, 0xC0, 0x00, 0x00, 0xA0, 0x40, 0x00, 0x00,
    0xC0, 0xC0, 0x24, 0x67};

void check(bool condition, const char* description)
{
    ++checks;
    if (!condition)
    {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

// Bitwise reference avoids sharing the production CRC lookup table.
uint16_t wire_crc(const uint8_t* data, std::size_t size)
{
    uint16_t result = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i)
    {
        result ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
        {
            result = static_cast<uint16_t>((result >> 1U) ^ ((result & 1U) ? 0x8408U : 0U));
        }
    }
    return result;
}

void update_crc(std::array<uint8_t, 28>& frame)
{
    const uint16_t value = wire_crc(frame.data(), 26);
    frame[26] = static_cast<uint8_t>(value);
    frame[27] = static_cast<uint8_t>(value >> 8U);
}

unsigned feed(vision_parser& parser, const uint8_t* data, std::size_t size, vision_command& command)
{
    unsigned received = 0;
    for (std::size_t i = 0; i < size; ++i)
    {
        if (parser.push(data[i], command))
        {
            ++received;
        }
    }
    return received;
}

bool matches_command(const vision_command& command)
{
    return command.mode == 7 && command.yaw == 0.5f && command.yaw_vel == -2.0f &&
           command.yaw_acc == 3.0f && command.pitch == -4.0f && command.pitch_vel == 5.0f &&
           command.pitch_acc == -6.0f;
}

void test_command_stream()
{
    check(wire_crc(command_frame.data(), 26) == 0x6724, "fixed command CRC is valid");
    vision_parser parser;
    vision_command command{};
    for (std::size_t i = 0; i < command_frame.size(); ++i)
    {
        check(parser.push(command_frame[i], command) == (i == command_frame.size() - 1),
            "command is delivered only after its final CRC byte");
    }
    check(matches_command(command), "command fields decode from little-endian wire values");

    for (std::size_t split = 1; split < command_frame.size(); ++split)
    {
        parser.reset();
        check(feed(parser, command_frame.data(), split, command) == 0,
            "partial USB chunk does not publish a command");
        check(feed(parser, command_frame.data() + split, command_frame.size() - split, command) ==
                      1 &&
                  matches_command(command),
            "every USB chunk split preserves the complete command");
    }

    std::vector<uint8_t> joined = {0x00, 0x5A, 0xFF, 0x42};
    joined.insert(joined.end(), command_frame.begin(), command_frame.end());
    joined.insert(joined.end(), command_frame.begin(), command_frame.end());
    parser.reset();
    check(feed(parser, joined.data(), joined.size(), command) == 2 && matches_command(command),
        "noise and two concatenated commands produce exactly two deliveries");

    parser.reset();
    feed(parser, command_frame.data(), 13, command);
    parser.reset();
    check(feed(parser, command_frame.data() + 13, command_frame.size() - 13, command) == 0,
        "reset discards an incomplete frame after a transport restart");
    check(feed(parser, command_frame.data(), command_frame.size(), command) == 1,
        "reset parser accepts the next full frame");
}

void test_resynchronization()
{
    vision_parser parser;
    vision_command command{99, 1, 2, 3, 4, 5, 6};
    auto damaged = command_frame;
    damaged[4] ^= 0x01;
    check(feed(parser, damaged.data(), damaged.size(), command) == 0,
        "payload corruption is rejected by CRC");
    check(command.mode == 99 && command.yaw == 1 && command.yaw_vel == 2 && command.yaw_acc == 3 &&
              command.pitch == 4 && command.pitch_vel == 5 && command.pitch_acc == 6,
        "rejected frame leaves the caller's command unchanged");
    check(feed(parser, command_frame.data(), command_frame.size(), command) == 1 &&
              matches_command(command),
        "valid frame immediately after CRC failure is recovered");

    // Exercise every possible position of a valid header inside a damaged candidate frame.
    for (std::size_t prefix = 1; prefix < command_frame.size(); ++prefix)
    {
        std::vector<uint8_t> stream(prefix, 0x69);
        stream[0] = 0xA5;
        stream.insert(stream.end(), command_frame.begin(), command_frame.end());
        parser.reset();
        check(feed(parser, stream.data(), stream.size(), command) == 1 && matches_command(command),
            "resynchronization preserves a valid header inside the rejected frame");
    }

    std::vector<uint8_t> false_headers = {0xA5, 0xA5, 0x00, 0xA5, 0x42, 0xA5};
    false_headers.insert(false_headers.end(), command_frame.begin(), command_frame.end());
    parser.reset();
    check(feed(parser, false_headers.data(), false_headers.size(), command) == 1 &&
              matches_command(command),
        "multiple false headers do not hide a following valid frame");
}

void test_nonfinite_commands()
{
    // +Inf, -Inf, quiet NaN, signaling NaN, and negative NaN as wire bit patterns.
    constexpr uint32_t nonfinite[] = {0x7F800000, 0xFF800000, 0x7FC00001, 0x7F800001, 0xFFC00001};
    for (std::size_t field = 0; field < 6; ++field)
    {
        for (const uint32_t bits : nonfinite)
        {
            auto frame = command_frame;
            for (unsigned byte = 0; byte < 4; ++byte)
            {
                frame[2 + field * 4 + byte] = static_cast<uint8_t>(bits >> (byte * 8));
            }
            update_crc(frame);
            vision_parser parser;
            vision_command command{99, 1, 2, 3, 4, 5, 6};
            check(feed(parser, frame.data(), frame.size(), command) == 0,
                "CRC-valid nonfinite value is rejected in every command field");
            check(command.mode == 99 && command.yaw == 1 && command.yaw_vel == 2 &&
                      command.yaw_acc == 3 && command.pitch == 4 && command.pitch_vel == 5 &&
                      command.pitch_acc == 6,
                "nonfinite frame does not partially overwrite the last command");
            check(feed(parser, command_frame.data(), command_frame.size(), command) == 1 &&
                      matches_command(command),
                "parser recovers immediately after rejecting a nonfinite value");
        }
    }
}

void test_telemetry()
{
    vision_telemetry telemetry{};
    telemetry.detect_color = true;
    telemetry.reset_tracker = true;
    telemetry.mode = 0x27;
    telemetry.q[0] = 1.0f;
    telemetry.q[1] = -0.5f;
    telemetry.q[2] = 0.25f;
    telemetry.q[3] = -0.125f;
    telemetry.yaw = 725.5f;
    telemetry.gyro_yaw = -3.0f;
    telemetry.pitch = -22.25f;
    telemetry.gyro_pitch = 4.5f;
    telemetry.bullet_speed = 30.5f;
    telemetry.bullet_count = 0x1234;

    // Angles remain degrees, including continuous yaw beyond one turn. All multibyte values
    // are little-endian; the reserved byte and trailing CRC are part of the fixed wire layout.
    constexpr std::array<uint8_t, 44> expected = {0x5A, 0x03, 0x27, 0x00, 0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x00, 0xBF, 0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x00, 0xBE, 0x00, 0x60, 0x35,
        0x44, 0x00, 0x00, 0x40, 0xC0, 0x00, 0x00, 0xB2, 0xC1, 0x00, 0x00, 0x90, 0x40, 0x00, 0x00,
        0xF4, 0x41, 0x34, 0x12, 0x06, 0x23};
    const auto encoded = encode_vision(telemetry);
    check(encoded.size() == 44, "telemetry occupies exactly 44 wire bytes");
    check(encoded == expected, "telemetry matches the fixed legacy little-endian layout");
    check(wire_crc(encoded.data(), 42) == 0x2306,
        "telemetry CRC covers all 42 header and payload bytes");

    for (unsigned flags = 0; flags < 4; ++flags)
    {
        telemetry.detect_color = (flags & 1U) != 0;
        telemetry.reset_tracker = (flags & 2U) != 0;
        const auto frame = encode_vision(telemetry);
        const uint16_t checksum = wire_crc(frame.data(), 42);
        check(frame[1] == flags && frame[3] == 0,
            "color and tracker flags occupy independent bits with zero reserved byte");
        check(frame[42] == static_cast<uint8_t>(checksum) &&
                  frame[43] == static_cast<uint8_t>(checksum >> 8U),
            "changed telemetry flags receive an updated little-endian CRC");
    }
}
} // namespace

int main()
{
    test_command_stream();
    test_resynchronization();
    test_nonfinite_commands();
    test_telemetry();
    std::printf("Vision protocol tests: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
