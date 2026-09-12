#pragma once

#include <cstdint>

#include "vexdash/protocol_types.h"
#include "vexdash/transport.h"

// DeviceMap: builds and sends DEVICE_MAP frames (0x0D, v1.1, protocol.md
// §5.12) -- a full snapshot of "which smart port has which device".
//
// Usage pattern:
//   DeviceMap devmap(transport);
//   devmap.begin();
//   devmap.add_port(3, DeviceType::kMotor, /*connected=*/true, "left_drive");
//   devmap.add_port(4, DeviceType::kRotation, true, "");
//   devmap.add_port(22, DeviceType::kOptical, false, "");  // ADI port A
//   devmap.send();   // encodes+sends, splitting across frames if needed
//
// The snapshot is complete (not incremental). One snapshot may need more than
// one frame, so the front-end rule is per-BATCH, not per-frame (§5.12): the
// FIRST frame of a batch replaces the whole port table, and continuation frames
// merge into it by port. A frame starts a new batch when it repeats a port
// already seen in the current batch (primary, clock-free) or arrives more than
// 500ms after the previous one (secondary). That way a split snapshot is not
// truncated to its last frame, and a port that disappears from a later snapshot
// still disappears from the table. Send the frames of ONE snapshot back-to-back.
// 中文：快照是完整的（非增量），但可能拆成多個 frame，所以前端是「以批次為單位」
// 取代：帶到本批已出現過的埠、或距上一幀超過 500ms＝新批次（整包取代），否則以
// port 為鍵合併。同一個快照的各幀務必連續送完，中間不要插入延遲。
// No dynamic allocation: the staging buffer for entries is a fixed-size member.

namespace vexdash {

// protocol.md §5.12: port numbering. 1-21 smart ports, 22-29 ADI A-H.
constexpr std::uint8_t kAdiPortBase = 22;  // port 22 == ADI 'A'

// Helper: ADI letter ('A'..'H' / 'a'..'h') -> wire port number (22..29),
// or 0 if out of range (0 is never a valid port, acts as sentinel).
constexpr std::uint8_t adi_port(char letter) {
  char up = (letter >= 'a' && letter <= 'z') ? static_cast<char>(letter - 'a' + 'A') : letter;
  if (up < 'A' || up > 'H') return 0;
  return static_cast<std::uint8_t>(kAdiPortBase + (up - 'A'));
}

// Max entries stageable before send(). Comfortably covers 21 smart + 8 ADI
// ports; sizing the staging buffer statically (no allocation).
constexpr std::size_t kMaxDeviceEntries = 32;

class DeviceMap {
 public:
  explicit DeviceMap(ITransport& transport);

  // Clears any staged entries to start building a fresh snapshot.
  void begin();

  // Stages one port entry. `name` may be "" (name_len=0). `port` uses the
  // §5.12 numbering (1-21 smart, 22-29 ADI). Returns false if the staging
  // buffer is full (kMaxDeviceEntries) or name exceeds 63 bytes -- the
  // entry is not staged in that case.
  bool add_port(std::uint8_t port, DeviceType device_type, bool connected, const char* name = "");

  // Encodes and sends the staged snapshot. If the entries don't fit in one
  // 512-byte frame (protocol.md §5.12), splits across multiple DEVICE_MAP
  // frames sent back-to-back; the front-end merges frames that arrive within
  // its batch window into one snapshot (see the batch rule at the top of this
  // file -- send the frames of one snapshot without delay). Returns false on any
  // encode/transport failure. Does NOT clear staged entries (call begin()
  // to reset); sending twice re-sends the same snapshot.
  bool send();

  std::size_t entry_count() const { return entry_count_; }

 private:
  struct Entry {
    std::uint8_t port;
    DeviceType device_type;
    bool connected;
    std::uint8_t name_len;
    char name[kMaxDeviceNameLen];
  };

  bool send_entries(std::size_t begin_idx, std::size_t end_idx);

  ITransport& transport_;
  Entry entries_[kMaxDeviceEntries];
  std::size_t entry_count_ = 0;
};

}  // namespace vexdash
