#pragma once
// XBOT / LebiTEC BLE protocol (read side). Ported from the open-source
// "esphome-xbot" component (components/xbot/*), including its captured test frame.
//
// STATUS OF EACH PART (see docs/PROTOCOL.md):
//  * Transport profiles, framing, checksum, XOR obfuscation, variant-from-name table,
//    read request blocks, register decoding: taken from that source.
//  * That source was verified on ONE 36 V vehicle. Whether it matches an
//    E-Wheels e2s Urban Max (48 V, controller a5.9.6) is NOT verified.
//  * Only reads are implemented here. No write frame is ever built by the firmware.
//
// Pure C++11, no Arduino dependency, so it is unit-tested on a PC.
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "Telemetry.h"

namespace xbot {

// ---- transport profiles (service, write, notify) -------------------------------------
struct TransportProfile {
  const char* name;
  const char* service;
  const char* write;
  const char* notify;
};
static const int kProfileCount = 5;
static const TransportProfile kProfiles[kProfileCount] = {
    {"nus", "6e400001-b5a3-f393-e0a9-e50e24dcca9e", "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
     "6e400003-b5a3-f393-e0a9-e50e24dcca9e"},
    {"ae00", "0000ae00-0000-1000-8000-00805f9b34fb", "0000ae01-0000-1000-8000-00805f9b34fb",
     "0000ae02-0000-1000-8000-00805f9b34fb"},
    {"ffe0", "0000ffe0-0000-1000-8000-00805f9b34fb", "0000fff3-0000-1000-8000-00805f9b34fb",
     "0000fff4-0000-1000-8000-00805f9b34fb"},
    {"fff0-f3f7", "0000fff0-0000-1000-8000-00805f9b34fb", "0000fff3-0000-1000-8000-00805f9b34fb",
     "0000fff7-0000-1000-8000-00805f9b34fb"},
    {"fff0-f2f1", "0000fff0-0000-1000-8000-00805f9b34fb", "0000fff2-0000-1000-8000-00805f9b34fb",
     "0000fff1-0000-1000-8000-00805f9b34fb"},
};

// ---- protocol variant, chosen by the advertised device name --------------------------
struct ProtocolVariant {
  uint8_t id;
  uint8_t xorKey;    // 0 = frames in the clear
  uint8_t readDest;
  uint8_t readCmd;
};
static const uint8_t kSupportedVariant = 3;

namespace detail {
struct NamePattern {
  const char* needle;
  ProtocolVariant v;
};
static const ProtocolVariant V3 = {3, 0x34, 0x20, 0x61};
static const ProtocolVariant V3_PLAIN = {3, 0x00, 0x20, 0x61};
static const NamePattern kEarly[] = {
    {"X0Robot", V3_PLAIN}, {"PRO-II", V3}, {"TK2-S", V3}, {"TK2_Y", V3}, {"M1 MAX", V3},
    {"X2", V3}, {"CURTIS", V3}, {"MSEnery", V3}, {"smart", V3},
    {"TK", {7, 0x00, 0x0A, 0x01}}, {"XRIDER", V3}, {"NXRIDE", V3},
    {"M0Clean", {6, 0x00, 0x32, 0x01}}, {"XBOT_AGV", {1, 0x15, 0x0A, 0x01}},
    {"M6", {0, 0x00, 0x0A, 0x01}}, {"MiniRobot", {1, 0x55, 0x0A, 0x01}},
    {"GoKart", {4, 0x00, 0x0A, 0x01}}, {"N3MTenbot", {1, 0x00, 0x0A, 0x01}},
    {" MiniPro", {1, 0x00, 0x0A, 0x01}}, {" Ninebot", {1, 0x00, 0x0A, 0x01}},
    {"Ninebot", {1, 0x00, 0x0A, 0x01}}, {"miniPLUS_", {1, 0x00, 0x04, 0x01}},
    {"M5Robot", {1, 0xD8, 0x0A, 0x01}},
};
static const NamePattern kLate[] = {
    {"A6Robot", {2, 0xD8, 0x0A, 0x01}}, {"M0", V3}, {"SFSO", V3}, {"MIScooter", V3_PLAIN},
    {"W1", {5, 0x37, 0x0A, 0x01}}, {"Plus", {1, 0x39, 0x04, 0x01}},
};
template <size_t N>
inline const NamePattern* match(const char* name, const NamePattern (&tab)[N]) {
  for (size_t i = 0; i < N; ++i)
    if (strstr(name, tab[i].needle)) return &tab[i];
  return 0;
}
}  // namespace detail

// Case-sensitive substring match, first hit wins; a name matching nothing is variant 3.
inline ProtocolVariant variantForName(const char* name) {
  if (!name) name = "";
  const detail::NamePattern* hit = detail::match(name, detail::kEarly);
  if (hit) return hit->v;
  const bool hasM1 = strstr(name, "M1") != 0;
  const char* x = strchr(name, 'X');
  const bool xEarly = x != 0 && (x - name) < 2;
  if (hasM1 || xEarly) {
    if (strlen(name) > 6) {
      ProtocolVariant v = {1, (uint8_t)(hasM1 ? 0x7A : 0x18), 0x0A, 0x01};
      return v;
    }
    return detail::V3;
  }
  hit = detail::match(name, detail::kLate);
  if (hit) return hit->v;
  return detail::V3;
}

// ---- framing:  55 AA <len+2> <dest> <cmd> <reg> <payload...> <~sum LE> --------------
static const size_t kMaxRequest = 16;

inline void applyXor(uint8_t* b, size_t n, uint8_t key) {
  if (key == 0) return;
  for (size_t i = 0; i < n; ++i) b[i] = (uint8_t)(b[i] ^ key);
}

inline bool checksumOk(const uint8_t* f, size_t n) {
  if (n < 7) return false;
  const size_t end = n - 2;
  uint16_t sum = 0;
  for (size_t i = 2; i < end; ++i) sum = (uint16_t)(sum + f[i]);
  const uint16_t want = (uint16_t)~sum;
  const uint16_t got = (uint16_t)(f[end] | (f[end + 1] << 8));
  return want == got;
}

// Returns the frame length, or 0 if `out` is too small.
inline size_t buildRequest(uint8_t* out, size_t cap, uint8_t dest, uint8_t cmd, uint8_t reg,
                           const uint8_t* payload, size_t plen) {
  const size_t total = plen + 8;
  if (cap < total) return 0;
  out[0] = 0x55;
  out[1] = 0xAA;
  out[2] = (uint8_t)(plen + 2);
  out[3] = dest;
  out[4] = cmd;
  out[5] = reg;
  for (size_t i = 0; i < plen; ++i) out[6 + i] = payload[i];
  uint16_t sum = 0;
  for (size_t i = 2; i < total - 2; ++i) sum = (uint16_t)(sum + out[i]);
  const uint16_t crc = (uint16_t)~sum;
  out[total - 2] = (uint8_t)(crc & 0xFF);
  out[total - 1] = (uint8_t)(crc >> 8);
  return total;
}

// WRITE frame: same layout, cmd 0x03, payload = 16-bit little-endian value.
static const uint8_t kDestMain = 0x20;
static const uint8_t kWriteCmd = 0x03;
inline size_t buildWrite(uint8_t* out, size_t cap, uint8_t dest, uint8_t cmd, uint8_t reg, int16_t value) {
  const uint16_t raw = (uint16_t)value;
  const uint8_t payload[2] = {(uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8)};
  return buildRequest(out, cap, dest, cmd, reg, payload, 2);
}
// A flag sharing its register with others: keep the bits that are not being changed.
inline uint16_t applyBit(uint16_t raw, uint16_t mask, bool on) {
  return on ? (uint16_t)(raw | mask) : (uint16_t)(raw & (uint16_t)~mask);
}

// Notifications are not frame-aligned. Calls fn(frame, len) for each complete,
// checksum-valid frame; returns how many leading bytes were consumed.
template <typename F>
inline size_t extractFrames(const uint8_t* buf, size_t n, F fn) {
  size_t pos = 0;
  for (;;) {
    while (pos + 1 < n && (buf[pos] != 0x55 || buf[pos + 1] != 0xAA)) ++pos;
    if (n - pos < 3) return pos;
    const size_t total = (size_t)buf[pos + 2] + 6;
    if (n - pos < total) return pos;
    if (checksumOk(buf + pos, total)) {
      fn(buf + pos, total);
      pos += total;
    } else {
      pos += 2;  // header bytes can occur inside a payload: resync
    }
  }
}

template <size_t CAP>
class RxAccumulator {
 public:
  RxAccumulator() : len_(0) {}
  void clear() { len_ = 0; }
  size_t size() const { return len_; }
  // De-obfuscates only the new bytes. Returns bytes thrown away to make room.
  size_t append(const uint8_t* data, size_t n, uint8_t key) {
    size_t dropped = 0;
    if (n > CAP - len_) { dropped = len_; len_ = 0; }
    if (n > CAP) return dropped;
    memcpy(buf_ + len_, data, n);
    applyXor(buf_ + len_, n, key);
    len_ += n;
    return dropped;
  }
  template <typename F>
  void drain(F fn) {
    const size_t used = extractFrames(buf_, len_, fn);
    if (used == 0) return;
    len_ -= used;
    memmove(buf_, buf_ + used, len_);
  }
 private:
  uint8_t buf_[CAP];
  size_t len_;
};

// Reply: 16-bit little-endian registers numbered upward from the frame's register byte.
template <typename F>
inline void decodeRegisters(const uint8_t* f, size_t n, F fn) {
  if (n < 8) return;
  const uint8_t base = f[5];
  const size_t payload = n - 8;
  for (size_t i = 0; i + 1 < payload; i += 2) {
    if ((size_t)base + (i / 2) > 0xFF) break;
    fn((uint8_t)(base + (i / 2)), (uint16_t)(f[6 + i] | (f[6 + i + 1] << 8)));
  }
}

// Share of distinct byte values, 0-100 (diagnostics only).
inline uint8_t byteDiversityPct(const uint8_t* b, size_t n) {
  if (n == 0) return 0;
  bool seen[256];
  memset(seen, 0, sizeof(seen));
  size_t d = 0;
  for (size_t i = 0; i < n; ++i) if (!seen[b[i]]) { seen[b[i]] = true; ++d; }
  return (uint8_t)((d * 100) / n);
}

// ---- read requests (one per block, in this order) ------------------------------------
struct PollEntry {
  uint8_t dest, cmd, reg, count;  // count = bytes asked for (payload byte)
};
static const int kPollCount = 20;
static const PollEntry kPollTable[kPollCount] = {
    {0x20, 0x61, 0xEE, 0x0C}, {0x20, 0x61, 0x46, 0x2A}, {0x20, 0x61, 0x1A, 0x34},
    {0x20, 0x61, 0x6E, 0x32}, {0x20, 0x61, 0xC2, 0x2C}, {0x20, 0x61, 0xEA, 0x20},
    {0x21, 0x01, 0x14, 0x2E}, {0x20, 0x61, 0x14, 0x1E}, {0x20, 0x61, 0x5A, 0x2A},
    {0x20, 0x61, 0x32, 0x32}, {0x20, 0x61, 0xBB, 0x06}, {0x20, 0x61, 0x58, 0x10},
    {0x20, 0x61, 0x2A, 0x22}, {0x20, 0x61, 0xBA, 0x0E}, {0x20, 0x61, 0x66, 0x0E},
    {0x20, 0x61, 0xA0, 0x3A}, {0x20, 0x61, 0xD4, 0x1A}, {0x20, 0x61, 0x7B, 0x0A},
    {0x20, 0x61, 0xF2, 0x06}, {0x20, 0x61, 0xFD, 0x06},
};
// After the first full sweep, the dashboard re-reads mostly the blocks holding speed
// (0x1A..) and voltage/current (0x46..), with occasional mode (0x6E) and light (0xF2).
static const int kLiveCount = 20;
static const uint8_t kLiveSeq[kLiveCount] = {2, 1, 2, 1, 2, 1, 2, 1, 2, 3, 2, 1, 2, 1, 2, 1, 2, 1, 2, 18};

static const uint8_t kSrcMain = 0x23;

// ---- register store -------------------------------------------------------------------
// seq() ties registers to the frame they arrived in (needed for 32-bit and product values).
class RegisterFile {
 public:
  RegisterFile() { clear(); }
  void clear() { memset(v_, 0, sizeof(v_)); memset(s_, 0, sizeof(s_)); now_ = 0; }
  void beginFrame() { if (++now_ == 0) now_ = 1; }
  void set(uint8_t r, uint16_t val) { v_[r] = val; s_[r] = now_; }
  bool has(uint8_t r) const { return s_[r] != 0; }
  uint16_t get(uint8_t r) const { return v_[r]; }
  uint16_t seq(uint8_t r) const { return s_[r]; }
 private:
  uint16_t v_[256];
  uint16_t s_[256];
  uint16_t now_;
};

// ---- register map -> dashboard telemetry ---------------------------------------------
// Scales and ranges are those of the reference component (one 36 V vehicle). A reading
// outside its range, or the 0xFFFF "not available" marker, is shown as "--", never clamped.
static const uint8_t REG_ERROR = 0x1B, REG_WARNING = 0x1C, REG_CTRL_VER_HI = 0x1E, REG_CTRL_VER_LO = 0x1D;
static const uint8_t REG_BATT = 0x22, REG_SPEED = 0x27, REG_TOTAL = 0x29, REG_TRIP = 0x2F;
static const uint8_t REG_VOLT = 0x47, REG_CURR = 0x48, REG_MODE = 0x7E;
static const uint8_t REG_ZERO_START = 0x7D, REG_LIGHT = 0xF2;  // bit 0x0001 of each register

// The ONLY registers this firmware may ever write. Speed limits, cruise control, lock and
// everything else are deliberately absent.
static const uint8_t REG_WHEEL_FACTOR = 0xEE, REG_LOCK = 0x7F;
static const uint8_t REG_LIM_SPORT = 0xF0, REG_LIM_DRIVE = 0xEF, REG_LIM_ECO = 0xF1;
inline bool isWritableReg(uint8_t reg) {
  return reg == REG_LIGHT || reg == REG_ZERO_START || reg == REG_MODE || reg == REG_LOCK ||
         reg == REG_LIM_SPORT || reg == REG_LIM_DRIVE || reg == REG_LIM_ECO;
}

// Speed limits are counts scaled by the wheel factor the scooter reports (reference component).
static const float kWheelFactorDivisor = 5794.65283203125f;
inline int limitMaxKmh(uint8_t reg) {  // slider ranges of the reference: sport 6-35, drive 6-30, eco 6-30
  return reg == REG_LIM_SPORT ? 35 : (reg == REG_LIM_DRIVE || reg == REG_LIM_ECO) ? 30 : 0;
}
inline bool scaleFromWheelFactor(uint16_t wf, float& scale) {
  if (wf == 0 || wf == 0xFFFF) return false;
  scale = kWheelFactorDivisor / (float)wf;  // counts per km/h
  return true;
}
inline int16_t countForKmh(float kmh, float scale) {  // truncated, as the reference does
  int32_t raw = (int32_t)(kmh * scale);
  if (raw < 0) raw = 0;
  if (raw > 32767) raw = 32767;
  return (int16_t)raw;
}
inline int kmhFromCount(uint16_t raw, float scale) { return (int)((float)raw / scale + 0.5f); }
inline bool limitCountAllowed(uint8_t reg, uint16_t raw, float scale) {
  const int mx = limitMaxKmh(reg);
  if (mx == 0 || scale <= 0.0f) return false;
  const int k = kmhFromCount(raw, scale);
  return k >= 6 && k <= mx;
}
// Locking is refused unless the scooter is known to stand still. Unlocking is always allowed.
inline bool lockAllowed(bool lockIt, bool speedValid, float kmh) {
  if (!lockIt) return true;
  return speedValid && kmh <= 0.5f;
}

// Riding mode register value = index into the reference list {Sport, Drive, Eco, Walking}.
// That order comes from the reference component (one vehicle).
static const char* const kModeNames[4] = {"SPORT", "DRIVE", "ECO", "WALK"};
// NOVA internal order is Walk=0, Eco=1, Drive=2, Sport=3.
inline uint8_t modeIndexFromInternal(uint8_t internalMode) { return (uint8_t)(3 - (internalMode & 3)); }
inline uint8_t internalFromModeIndex(uint8_t idx) { return (uint8_t)(3 - (idx & 3)); }

// What the scooter itself reports (read back), never what was requested.
struct Readback {
  bool lightValid, light, zeroValid, zero, modeValid;
  uint8_t modeIdx;
  bool lockValid, locked;
  bool limValid[3];  // order: sport (0xF0), drive (0xEF), eco (0xF1)
  int limKmh[3];
  Readback() : lightValid(false), light(false), zeroValid(false), zero(false), modeValid(false), modeIdx(0),
               lockValid(false), locked(false) {
    for (int i = 0; i < 3; ++i) { limValid[i] = false; limKmh[i] = 0; }
  }
};
// 1 = every mode limit read back at or below capKmh, 0 = at least one is higher, -1 = unknown.
inline int limitsCapped(const Readback& b, int capKmh) {
  int r = 1;
  for (int i = 0; i < 3; ++i) {
    if (!b.limValid[i]) return -1;
    if (b.limKmh[i] > capKmh) r = 0;
  }
  return r;
}


inline Readback readbackFrom(const RegisterFile& r) {
  Readback b;
  if (r.has(REG_LIGHT) && r.get(REG_LIGHT) != 0xFFFF) { b.lightValid = true; b.light = (r.get(REG_LIGHT) & 1) != 0; }
  if (r.has(REG_ZERO_START) && r.get(REG_ZERO_START) != 0xFFFF) { b.zeroValid = true; b.zero = (r.get(REG_ZERO_START) & 1) != 0; }
  if (r.has(REG_MODE) && r.get(REG_MODE) < 4) { b.modeValid = true; b.modeIdx = (uint8_t)r.get(REG_MODE); }
  if (r.has(REG_LOCK) && r.get(REG_LOCK) != 0xFFFF) { b.lockValid = true; b.locked = r.get(REG_LOCK) != 0; }
  float scale;
  if (r.has(REG_WHEEL_FACTOR) && scaleFromWheelFactor(r.get(REG_WHEEL_FACTOR), scale)) {
    const uint8_t regs[3] = {REG_LIM_SPORT, REG_LIM_DRIVE, REG_LIM_ECO};
    for (int i = 0; i < 3; ++i)
      if (r.has(regs[i]) && limitCountAllowed(regs[i], r.get(regs[i]), scale)) {
        b.limValid[i] = true;
        b.limKmh[i] = kmhFromCount(r.get(regs[i]), scale);
      }
  }
  return b;
}

inline void fillTelemetry(const RegisterFile& r, Telemetry& t) {
  t = Telemetry();
  if (r.has(REG_SPEED) && r.get(REG_SPEED) != 0xFFFF) {
    const float v = (float)r.get(REG_SPEED) / 100.0f;
    if (v <= 150.0f) { t.speedValid = true; t.speedKmh = v; }
  }
  if (r.has(REG_BATT) && r.get(REG_BATT) <= 100) { t.battPctValid = true; t.battPct = (uint8_t)r.get(REG_BATT); }
  bool voltOk = false;
  if (r.has(REG_VOLT) && r.get(REG_VOLT) != 0xFFFF) {
    const float v = (float)r.get(REG_VOLT) / 100.0f;
    if (v <= 120.0f) { t.voltValid = true; t.volts = v; voltOk = true; }
  }
  if (voltOk && r.has(REG_CURR) && r.seq(REG_CURR) == r.seq(REG_VOLT)) {  // 0xFFFF is a legal -1 here
    const float w = (float)r.get(REG_VOLT) * (float)(int16_t)r.get(REG_CURR) / 10000.0f;
    if (w >= -20000.0f && w <= 20000.0f) { t.powerValid = true; t.powerW = (int)(w + (w >= 0 ? 0.5f : -0.5f)); }
  }
  if (r.has(REG_TRIP) && r.get(REG_TRIP) != 0xFFFF) {
    const float v = (float)(int16_t)r.get(REG_TRIP) / 100.0f;
    if (v >= 0.0f && v <= 330.0f) { t.tripValid = true; t.tripKm = v; }
  }
  if (r.has(REG_TOTAL) && r.has(REG_TOTAL + 1) && r.seq(REG_TOTAL) == r.seq(REG_TOTAL + 1)) {
    const uint32_t raw = ((uint32_t)r.get(REG_TOTAL + 1) << 16) | r.get(REG_TOTAL);
    const float v = (float)raw / 1000.0f;
    if (v <= 200000.0f) { t.totalValid = true; t.totalKm = v; }
  }
  if (r.has(REG_ERROR) && r.get(REG_ERROR) != 0xFFFF) { t.faultValid = true; t.faultCode = r.get(REG_ERROR); }
  // Names follow the reference component's list (Sport, Drive, Eco, Walking).
  if (r.has(REG_MODE) && r.get(REG_MODE) < 4) {
    t.modeValid = true;
    snprintf(t.mode, sizeof(t.mode), "%s", kModeNames[r.get(REG_MODE)]);
  }
}

// Controller version as the reference renders it: registers 0x1E then 0x1D as hex.
inline bool controllerVersion(const RegisterFile& r, char* out, size_t n) {
  if (!r.has(REG_CTRL_VER_HI) || !r.has(REG_CTRL_VER_LO)) return false;
  snprintf(out, n, "%04X%04X", (unsigned)r.get(REG_CTRL_VER_HI), (unsigned)r.get(REG_CTRL_VER_LO));
  return true;
}

// Diagnosis only: which documented key would turn a raw notification into a 55 AA header?
// Returns the key (0 = already in the clear), or -1 if none of them does.
inline int diagnoseKey(const uint8_t* raw, size_t n) {
  static const uint8_t keys[] = {0x00, 0x34, 0x15, 0x55, 0xD8, 0x7A, 0x18, 0x37, 0x39};
  if (n < 2) return -1;
  for (size_t i = 0; i < sizeof(keys); ++i)
    if ((uint8_t)(raw[0] ^ keys[i]) == 0x55 && (uint8_t)(raw[1] ^ keys[i]) == 0xAA) return keys[i];
  return -1;
}

}  // namespace xbot
