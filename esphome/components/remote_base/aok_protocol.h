#pragma once

#include <bitset>
#include "remote_base.h"

namespace esphome {
namespace remote_base {

enum AOKCommand : uint8_t {
  COMMAND_UP      = 0x0b,
  COMMAND_DOWN    = 0x43,
  COMMAND_STOP    = 0x23,
  COMMAND_PROGRAM = 0x53,
  COMMAND_RELEASE = 0x24,  // NOTE: The RELEASE command is only transmitted after the UP and DOWN buttons are released.
};

enum AOKChannel : uint16_t {
  CHANNEL_1   = 0x0100,
  CHANNEL_2   = 0x0200,
  CHANNEL_3   = 0x0400,
  CHANNEL_4   = 0x0800,
  CHANNEL_5   = 0x1000,
  CHANNEL_6   = 0x2000,
  CHANNEL_7   = 0x4000,
  CHANNEL_8   = 0x8000,
  CHANNEL_9   = 0x0001,
  CHANNEL_10  = 0x0002,
  CHANNEL_11  = 0x0004,
  CHANNEL_12  = 0x0008,
  CHANNEL_13  = 0x0010,
  CHANNEL_14  = 0x0020,
  CHANNEL_15  = 0x0040,
  CHANNEL_16  = 0x0080,
  CHANNEL_ALL = 0xffff,
};

struct AOKData {
  uint32_t device;
  AOKChannel channel;
  AOKCommand command;

  uint8_t checksum() const {
    uint16_t sum = 0;
    const uint8_t* device_ptr = reinterpret_cast<const uint8_t*>(&device);
    const uint8_t* channel_ptr = reinterpret_cast<const uint8_t*>(&channel);
    sum += device_ptr[0] + device_ptr[1] + device_ptr[2];
    sum += channel_ptr[0] + channel_ptr[1];
    sum += command;
    return static_cast<uint8_t>(sum & 0xFF);
  }

  bool operator==(const AOKData &rhs) const {
    return device == rhs.device && channel == rhs.channel && command == rhs.command;
  }
};

class AOKProtocol : public RemoteProtocol<AOKData> {
 private:
  auto packet_bits(const AOKData &data) const;
  void encode_packet(RemoteTransmitData *dst, const AOKData &data) const;
  optional<AOKData> decode_packet(RemoteReceiveData &src) const;

 public:
  void encode(RemoteTransmitData *dst, const AOKData &data) override;
  optional<AOKData> decode(RemoteReceiveData src) override;
  void dump(const AOKData &data) override;
};

DECLARE_REMOTE_PROTOCOL(AOK)

template<typename... Ts> class AOKAction : public RemoteTransmitterActionBase<Ts...> {
 public:
  TEMPLATABLE_VALUE(uint32_t, device)
  TEMPLATABLE_VALUE(uint16_t, channel)
  TEMPLATABLE_VALUE(uint8_t, command)

  void encode(RemoteTransmitData *dst, Ts... x) {
    AOKData data{};
    data.device = this->device_.value(x...);
    data.channel = static_cast<AOKChannel>(this->channel_.value(x...));
    data.command = static_cast<AOKCommand>(this->command_.value(x...));
    AOKProtocol().encode(dst, data);
  }
};

}  // namespace remote_base
}  // namespace esphome