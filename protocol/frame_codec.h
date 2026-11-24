// frame_codec.h —— 控制帧协议 v1 封装/解析（header-only）
// 与 protocol/protocol.md 严格一致；小端字节序；1 字节累加和取反校验。
// 供开发板 flight_controller 发送控制帧使用，亦可做对称解析。
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace control_frame {

// ---- 帧常量 ----
constexpr uint8_t  HEADER0    = 0xA5;   // 帧头
constexpr uint8_t  HEADER1    = 0xDA;
constexpr uint8_t  FIXED0     = 0x00;   // 偏移 8
constexpr uint8_t  FIXED1     = 0x01;   // 偏移 9
constexpr size_t   FIXED_LEN  = 12;     // 帧头10B + cmd1B + 校验1B

constexpr size_t   OFF_LEN    = 2;      // 整帧长度字段
constexpr size_t   OFF_TICK   = 4;      // 时标字段（uint32，从 0 递增，溢出回 0）
constexpr size_t   OFF_CMD    = 10;     // cmd 字段

// ---- 命令类型 ----
enum class Cmd : uint8_t {
    Velocity   = 0x01,   // 速度指令
    Waypoints  = 0x02,   // 航迹点
    Hover      = 0x03,   // 悬停
    ModeSwitch = 0x04,   // 模式切换
};

// ---- 模式切换标志 ----
enum class ModeFlag : uint8_t {
    ResumeTrajectory = 0x00,   // 恢复航迹
    InterruptToTrack = 0x01,   // 中断航迹，进入跟踪
};

// ---- 小端写入/读取 ----
inline void put_u16(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back(static_cast<uint8_t>(x & 0xFF));
    v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
}
inline void put_u32(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back(static_cast<uint8_t>(x & 0xFF));
    v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
    v.push_back(static_cast<uint8_t>((x >> 16) & 0xFF));
    v.push_back(static_cast<uint8_t>((x >> 24) & 0xFF));
}
inline void put_f32(std::vector<uint8_t>& v, float x) {
    uint32_t bits;
    std::memcpy(&bits, &x, 4);
    put_u32(v, bits);
}
inline uint16_t get_u16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
inline uint32_t get_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}
inline float get_f32(const uint8_t* p) {
    uint32_t b = get_u32(p);
    float f;
    std::memcpy(&f, &b, 4);
    return f;
}

// ---- 校验：累加和取反 ----
inline uint8_t checksum(const uint8_t* data, size_t n) {
    uint8_t sum = 0;
    for (size_t i = 0; i < n; ++i) sum = static_cast<uint8_t>(sum + data[i]);
    return static_cast<uint8_t>(-sum);   // 等价 -(sum) & 0xFF
}

// ---- 通用组帧 ----
inline std::vector<uint8_t> buildFrame(uint32_t tick, uint8_t cmd,
                                       const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> f;
    f.reserve(FIXED_LEN + payload.size());
    f.push_back(HEADER0);
    f.push_back(HEADER1);
    put_u16(f, static_cast<uint16_t>(FIXED_LEN + payload.size()));  // 整帧长度
    put_u32(f, tick);
    f.push_back(FIXED0);
    f.push_back(FIXED1);
    f.push_back(cmd);
    f.insert(f.end(), payload.begin(), payload.end());
    f.push_back(checksum(f.data(), f.size()));   // 校验覆盖前面全部字节
    return f;
}

// ---- 各命令组帧 ----
inline std::vector<uint8_t> buildVelocity(uint32_t tick,
                                          float vx, float vy, float vz, float vyaw) {
    std::vector<uint8_t> p;
    p.reserve(16);
    put_f32(p, vx);
    put_f32(p, vy);
    put_f32(p, vz);
    put_f32(p, vyaw);
    return buildFrame(tick, static_cast<uint8_t>(Cmd::Velocity), p);
}

inline std::vector<uint8_t> buildHover(uint32_t tick) {
    return buildFrame(tick, static_cast<uint8_t>(Cmd::Hover), {});
}

inline std::vector<uint8_t> buildModeSwitch(uint32_t tick, ModeFlag flag) {
    std::vector<uint8_t> p{static_cast<uint8_t>(flag)};
    return buildFrame(tick, static_cast<uint8_t>(Cmd::ModeSwitch), p);
}

inline std::vector<uint8_t> buildWaypoints(
    uint32_t tick, const std::vector<std::array<float, 3>>& pts) {
    std::vector<uint8_t> p;
    p.reserve(1 + pts.size() * 12);
    p.push_back(static_cast<uint8_t>(pts.size()));
    for (const auto& pt : pts) {
        put_f32(p, pt[0]);
        put_f32(p, pt[1]);
        put_f32(p, pt[2]);
    }
    return buildFrame(tick, static_cast<uint8_t>(Cmd::Waypoints), p);
}

// ---- 解析结果 ----
struct ParsedFrame {
    bool ok = false;
    uint8_t cmd = 0;
    uint32_t tick = 0;
    std::vector<uint8_t> payload;   // 载荷数据（不含 cmd 与校验）
    std::string err;
};

// ---- 解析 ----
inline ParsedFrame parse(const uint8_t* data, size_t n) {
    ParsedFrame r;
    if (n < FIXED_LEN) { r.err = "frame too short"; return r; }
    if (data[0] != HEADER0 || data[1] != HEADER1) { r.err = "bad header"; return r; }

    const uint16_t len = get_u16(data + OFF_LEN);
    if (len != n) { r.err = "length mismatch"; return r; }

    uint8_t sum = 0;
    for (size_t i = 0; i < n; ++i) sum = static_cast<uint8_t>(sum + data[i]);
    if (sum != 0) { r.err = "checksum failed"; return r; }

    r.tick = get_u32(data + OFF_TICK);
    r.cmd  = data[OFF_CMD];
    r.payload.assign(data + 11, data + n - 1);   // 偏移 11 … n-2
    r.ok = true;
    return r;
}

}  // namespace control_frame
