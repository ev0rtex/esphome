#include "aok_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace remote_base {

static const char *const AOK_TAG = "remote.aok";

// AOK protocol constants
static const uint8_t AOK_PACKET_SIZE = 64;
static const uint8_t AOK_HEADER = 0xa3;
static const uint16_t AOK_PRE_POST_ZEROS = 8 * 2; // Most remotes (not old ones) usually do 7-8 zeros for a preamble.
                                                  // Adding extras as well as a postamble seems to improve reliability
                                                  // with OOK modules like the STX882.
static const uint16_t AOK_PACKET_PREFIX_MARK  = 5000;
static const uint16_t AOK_PACKET_SUFFIX_SPACE = 5000;
static const uint16_t AOK_ONE_MARK   = 600;
static const uint16_t AOK_ONE_SPACE  = 275;
static const uint16_t AOK_ZERO_MARK  = 290;
static const uint16_t AOK_ZERO_SPACE = 600;
static const uint16_t AOK_REPEATS = 6;

auto AOKProtocol::packet_bits(const AOKData &data) const {
  std::bitset<AOK_PACKET_SIZE> bits(0);
  bits |= (uint64_t) AOK_HEADER << 56;
  bits |= (uint64_t) data.device << 32;
  bits |= (uint64_t) data.channel << 16;
  bits |= (uint64_t) data.command << 8;
  bits |= (uint64_t) data.checksum();
  return bits;
}

void AOKProtocol::encode_packet(RemoteTransmitData *dst, const AOKData &data) const {
  // Packet prefix
  dst->item(AOK_PACKET_PREFIX_MARK, AOK_ZERO_SPACE);

  // Packet data
  auto bits = packet_bits(data);
  for (uint8_t i = 0; i < AOK_PACKET_SIZE; ++i) {
    if (bits[bits.size() - i - 1]) {
      dst->item(AOK_ONE_MARK, AOK_ONE_SPACE);
    } else {
      dst->item(AOK_ZERO_MARK, AOK_ZERO_SPACE);
    }
  }

  // Packet suffix
  dst->item(AOK_ONE_MARK, AOK_PACKET_SUFFIX_SPACE);
}

void AOKProtocol::encode(RemoteTransmitData *dst, const AOKData &data) {
  // Since we're using OOK, we don't need to set a carrier frequency
  dst->set_carrier_frequency(0);

  // Calculate the size of the data we're going to generate
  uint8_t command_count = data.command == COMMAND_UP || data.command == COMMAND_DOWN ? 2 : 1;
  uint16_t data_size = (
    AOK_PRE_POST_ZEROS +
    (((1 + AOK_PACKET_SIZE + 1) * command_count) * AOK_REPEATS) +
    AOK_PRE_POST_ZEROS
  ) * 2;
  dst->reserve(data_size);

  // Generate preamble
  for (uint8_t bit = 0; bit < AOK_PRE_POST_ZEROS; ++bit) {
    dst->item(AOK_ZERO_MARK, AOK_ZERO_SPACE);
  }

  // Generate the packet and repeat it
  for (uint8_t repeat = 0; repeat < AOK_REPEATS; ++repeat) {
    encode_packet(dst, data);
  }
  // If we're sending a command that requires a release command, send that as well
  if (data.command == COMMAND_UP || data.command == COMMAND_DOWN) {
    AOKData data_release = data;
    data_release.command = COMMAND_RELEASE;
    for (uint8_t repeat = 0; repeat < AOK_REPEATS; ++repeat) {
      encode_packet(dst, data_release);
    }
  }

  // Generate postamble (seems to improve reliability)
  for (uint8_t bit = 0; bit < AOK_PRE_POST_ZEROS; ++bit) {
    dst->item(AOK_ZERO_MARK, AOK_ZERO_SPACE);
  }
}

optional<AOKData> AOKProtocol::decode_packet(RemoteReceiveData &src) const {
  // Check for a packet prefix
  if (!src.expect_item(AOK_PACKET_PREFIX_MARK, AOK_ZERO_SPACE)) {
    ESP_LOGD(AOK_TAG, "Failed to find packet prefix");
    return {};
  }

  // Capture the packet bits
  std::bitset<AOK_PACKET_SIZE> bits(0);
  for (uint8_t i = 0; i < AOK_PACKET_SIZE; ++i) {
    if (src.expect_item(AOK_ONE_MARK, AOK_ONE_SPACE)) {
      bits[bits.size() - i - 1] = 1;
    } else if (src.expect_item(AOK_ZERO_MARK, AOK_ZERO_SPACE)) {
      bits[bits.size() - i - 1] = 0;
    } else {
      return {};
    }
  }

  // Check for a packet suffix
  if (!src.expect_item(AOK_ONE_MARK, AOK_PACKET_SUFFIX_SPACE)) {
    ESP_LOGD(AOK_TAG, "Failed to find packet suffix");
    return {};
  }

  // Convert to a uint64_t in one place for efficiency
  uint64_t ulbits = bits.to_ullong();

  // Check for the header
  if (static_cast<uint8_t>(ulbits >> 56) != AOK_HEADER) {
    ESP_LOGD(AOK_TAG, "Failed to find packet header");
    return {};
  }

  // Extract the fields
  AOKData data;
  data.device = static_cast<uint32_t>(ulbits >> 32 & 0xffffff);
  data.channel = static_cast<AOKChannel>(ulbits >> 16 & 0xffff);
  data.command = static_cast<AOKCommand>(ulbits >> 8 & 0xff);

  // Validate the checksum
  if (data.checksum() != static_cast<uint8_t>(ulbits & 0xff)) {
    ESP_LOGD(AOK_TAG, "Failed to validate checksum");
    return {};
  }

  return data;
}

optional<AOKData> AOKProtocol::decode(RemoteReceiveData src) {
  // We should have a minimum of 1 packet plus it's prefix and suffix
  const uint8_t min_size = 2 + (AOK_PACKET_SIZE * 2) + 2;

  // Sanity check... if we don't have enough data, there's no point in continuing
  if (src.size() < (min_size * AOK_REPEATS) + (AOK_PRE_POST_ZEROS * 2)) {
    return {};
  }

  std::vector<AOKData> packets;

  // We _can_ have up to AOK_REPEATS packets or even 2x AOK_REPEATS packets in a single transmission
  packets.reserve(AOK_REPEATS);

  // Loop through the data looking for AOK packets
  while (src.get_index() < src.size() - min_size) {
    // Skip data until we find a packet prefix and fix alignment if necessary (the data tends to be a bit noisy)
    uint16_t skipped = 0;
    while (!src.peek_item(AOK_PACKET_PREFIX_MARK, AOK_ZERO_SPACE)) {
      if (src.peek_item(AOK_PACKET_PREFIX_MARK, AOK_ZERO_SPACE, 1)) {
        ESP_LOGD(AOK_TAG, "Found misaligned item: %d, %d ... adjusting offset", src.peek(0), src.peek(1));
        src.advance(1);
      } else {
        src.advance(2);
        ++skipped;
      }

      // Just bail early if there's not enough data left to find a packet
      if (src.get_index() > src.size() - min_size) {
        return {};
      }
    }
    if (skipped > 0) {
      ESP_LOGD(AOK_TAG, "Skipped %d bits searching for AOK data", skipped);
    }

    // Try to decode the packet
    auto packet = decode_packet(src);
    if (packet.has_value()) {
      packets.push_back(packet.value());
      ESP_LOGD(AOK_TAG, "AOK Packet: device_id=0x%06x, channel_id=0x%04x, command_id=0x%02x, checksum=0x%02x",
              packet->device, packet->channel, packet->command, packet->checksum());
    }
  }

  if (packets.empty()) {
    return {};
  }
  return packets.front();
}

void AOKProtocol::dump(const AOKData &data) {
  ESP_LOGD(AOK_TAG, "AOK: device_id=0x%06x, channel_id=0x%04x, command_id=0x%02x, checksum=0x%02x",
           data.device, data.channel, data.command, data.checksum());
}

}  // namespace remote_base
}  // namespace esphome