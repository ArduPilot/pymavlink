static void emit(const mavlink_message_t *msg, unsigned pattern, unsigned version,
                 uint32_t source, uint32_t target, unsigned signed_frame)
{
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(bytes, msg);
    printf("%u %u %u %u %u %u ", msg->msgid, pattern, version, source, target, signed_frame);
    for (unsigned i = 0; i < length; i++) printf("%02x", bytes[i]);
    puts("");
}

static int verify_file(const char *name)
{
    FILE *file = fopen(name, "r");
    assert(file);
    char line[1024];
    unsigned count = 0;
    while (fgets(line, sizeof(line), file)) {
        unsigned id, version, source, target, signed_frame, length;
        char hex[600];
        assert(sscanf(line, "%u %u %u %u %u %u %599s", &id, &version, &source, &target, &signed_frame, &length, hex) == 7);
        assert(strlen(hex) == length * 2);
        mavlink_message_t rx = {0}, msg = {0};
        mavlink_status_t status = {0}, output = {0};
        mavlink_signing_t signing = {0};
        mavlink_signing_streams_t streams = {0};
        if (signed_frame) {
            memset(signing.secret_key, 42, 32);
            status.signing = &signing;
            status.signing_streams = &streams;
        }
        for (unsigned i=0; i<length; i++) {
            unsigned byte;
            assert(sscanf(hex+2*i, "%2x", &byte) == 1);
            uint8_t result = mavlink_frame_char_buffer(&rx, &status, byte, &msg, &output);
            assert(result == (i+1 == length ? MAVLINK_FRAMING_OK : MAVLINK_FRAMING_INCOMPLETE));
        }
        assert(msg.msgid == id && msg.sysid == source && msg.compid == 11);
        assert(msg.magic == (version == 1 ? MAVLINK_STX_MAVLINK1 : MAVLINK_STX));
        assert(mavlink_msg_get_target_sysid(&msg, mavlink_get_msg_entry(id)) == target);
        assert(!!(msg.incompat_flags & MAVLINK_IFLAG_SIGNED) == signed_frame);
        count++;
    }
    assert(count);
    fclose(file);
    printf("C verified %u Rust frames\n", count);
    return 0;
}
