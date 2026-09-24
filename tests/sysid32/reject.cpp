#include <cassert>
#include <fstream>
#include <iterator>
#include <vector>
#include "common/common.hpp"
namespace mavlink {
const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t id) {
    for (auto &entry : common::MESSAGE_ENTRIES) if (entry.msgid == id) return &entry;
    return nullptr;
}
}
int main(int argc, char **argv) {
    assert(argc == 2);
    for (unsigned flags=2; flags<=7; flags++) for (auto length : {0,80,255}) {
        std::ifstream file(std::string(argv[1])+"/"+std::to_string(flags)+"-"+std::to_string(length)+".v2", std::ios::binary);
        std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), {});
        assert(!bytes.empty());
        mavlink::mavlink_message_t rx = {}, msg = {};
        mavlink::mavlink_status_t status = {}, out = {};
        unsigned rejected = 0, accepted = 0;
        // Signatures are not authenticated in this framing-only test.
        for (auto byte : bytes) {
            if (mavlink::mavlink_frame_char_buffer(&rx, &status, byte, &msg, &out) != mavlink::MAVLINK_FRAMING_OK) continue;
            mavlink::MsgMap map(&msg);
            try {
                if (msg.msgid == 76) {
                    mavlink::common::msg::COMMAND_LONG command;
                    command.deserialize(map);
                    assert(false);
                } else {
                    mavlink::minimal::msg::HEARTBEAT heartbeat;
                    heartbeat.deserialize(map);
                    assert(heartbeat.base_mode == 81);
                    accepted++;
                }
            } catch (const std::runtime_error &) { rejected++; }
        }
        assert(rejected == 1 && accepted == 1);
    }
}
