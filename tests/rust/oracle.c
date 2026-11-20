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

// Run the C convenience parser, whose CRC failure path reuses a failing FD byte.
static int recover_file(const char *name)
{
    FILE *file = fopen(name, "rb");
    assert(file);
    mavlink_message_t msg = {0};
    mavlink_status_t status = {0};
    unsigned offset = 0;
    int byte;
    while ((byte = fgetc(file)) != EOF) {
        offset++;
        if (mavlink_parse_char(0, byte, &msg, &status) == MAVLINK_FRAMING_OK) {
            uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
            const uint16_t length = mavlink_msg_to_send_buffer(bytes, &msg);
            printf("%u ", offset);
            for (unsigned i = 0; i < length; i++) printf("%02x", bytes[i]);
            puts("");
        }
    }
    puts("");
    fclose(file);
    return 0;
}

static int default_heartbeats(void)
{
    for (unsigned version = 1; version <= 2; version++) {
        mavlink_message_t msg = {0};
        mavlink_status_t status = {0};
        status.current_tx_seq = 239;
        if (version == 1) status.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        mavlink_msg_heartbeat_pack_status(42, 11, &status, &msg, 0, 0, 0, 0, 0);
        emit(&msg, 0, version, 42, 0, 0);
    }
    return 0;
}
