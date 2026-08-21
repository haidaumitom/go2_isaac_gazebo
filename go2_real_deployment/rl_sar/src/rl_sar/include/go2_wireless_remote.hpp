#ifndef GO2_WIRELESS_REMOTE_HPP
#define GO2_WIRELESS_REMOTE_HPP

#include <unitree/idl/go2/WirelessController_.hpp>

#include <array>
#include <cstdint>
#include <cstring>

namespace go2_wireless_remote
{
struct RawPacket
{
    std::array<uint8_t, 2> head;
    uint16_t keys;
    float lx;
    float rx;
    float ry;
    float l2;
    float ly;
    std::array<uint8_t, 16> reserved;
};

static_assert(sizeof(RawPacket) == 40, "Unexpected Go2 wireless packet layout");

inline unitree_go::msg::dds_::WirelessController_ Decode(
    const std::array<uint8_t, 40>& bytes)
{
    RawPacket packet{};
    std::memcpy(&packet, bytes.data(), bytes.size());
    return unitree_go::msg::dds_::WirelessController_(
        packet.lx,
        packet.ly,
        packet.rx,
        packet.ry,
        packet.keys);
}
}

#endif  // GO2_WIRELESS_REMOTE_HPP
