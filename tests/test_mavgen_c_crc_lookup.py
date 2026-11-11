"""Exercise CRC lookup and frame parsing in generated headers."""
import shutil
import subprocess

import pytest

from pymavlink.generator import mavgen


@pytest.mark.parametrize('message_ids', [[], [17], [0, 17, 0xFFFFFF]])
@pytest.mark.parametrize('language', ['c', 'cpp'])
def test_crc_lookup_table_boundaries(tmp_path, message_ids, language):
    compiler = shutil.which('gcc' if language == 'c' else 'g++')
    if compiler is None:
        pytest.skip('GNU compiler is required to test generated headers')
    messages = ''.join(
        '<message id="%d" name="PROBE_%d"><description>CRC lookup probe.</description>'
        '<field type="uint8_t" name="value">Value.</field></message>' % (msgid, msgid)
        for msgid in message_ids
    )
    xml = tmp_path / 'lookup.xml'
    xml.write_text('<mavlink><version>3</version><messages>' + messages + '</messages></mavlink>')
    generated = tmp_path / 'generated'
    assert mavgen.mavgen(mavgen.Opts(output=str(generated), language='C',
                                   wire_protocol='2.0', validate=True), [str(xml)])
    checks = []
    for msgid in [0, 1, 16, 17, 18, 254, 255, 256, 65535, 0xFFFFFE, 0xFFFFFF, 0xFFFFFFFF]:
        checks.append(
            'e = mavlink_get_msg_entry(%du);\n'
            'if ((e != NULL) != %d) return 1;\n'
            'if (e && e->msgid != %du) return 2;\n'
            % (msgid, int(msgid in message_ids), msgid)
        )
    for v1 in [0, 1]:
        for msgid in [0, 17, 255] + ([] if v1 else [0xFFFFFF]):
            checks.append('if (parse_frame(%du, %d, %d)) return 3;\n' %
                          (msgid, int(msgid in message_ids), v1))
    source = tmp_path / ('lookup.' + language)
    source.write_text('''#include "lookup/mavlink.h"
static int parse_frame(uint32_t msgid, int known, int v1) {
    mavlink_message_t message, received, buffer;
    mavlink_status_t status, received_status;
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    mavlink_status_t *out_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    memset(&message, 0, sizeof(message));
    memset(&buffer, 0, sizeof(buffer));
    memset(&status, 0, sizeof(status));
    memset(&received_status, 0, sizeof(received_status));
    memset(out_status, 0, sizeof(*out_status));
    if (v1) out_status->flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    message.msgid = msgid;
    _MAV_PAYLOAD_NON_CONST(&message)[0] = 42;
    mavlink_finalize_message_chan(&message, 1, 1, MAVLINK_COMM_0, 1, 1,
                                 entry ? entry->crc_extra : 0);
    uint16_t length = mavlink_msg_to_send_buffer(bytes, &message);
    if (!length) return 1;
    uint8_t result = MAVLINK_FRAMING_INCOMPLETE;
    for (uint16_t i = 0; i < length; ++i) {
        result = mavlink_frame_char_buffer(&buffer, &status, bytes[i],
                                          &received, &received_status);
    }
    return result != (known ? MAVLINK_FRAMING_OK : MAVLINK_FRAMING_BAD_CRC);
}
int main(void) {
    const mavlink_msg_entry_t *e;
''' + ''.join(checks) + 'return 0; }\n')
    binary = tmp_path / 'lookup'
    build = subprocess.run(
        [compiler, '-std=c99' if language == 'c' else '-std=c++11',
         '-Wall', '-Wextra', '-Werror', '-Wno-address-of-packed-member',
         '-I' + str(generated), str(source), '-o', str(binary)],
        text=True, capture_output=True, timeout=30,
    )
    assert build.returncode == 0, build.stdout + build.stderr
    run = subprocess.run([str(binary)], text=True, capture_output=True, timeout=5)
    assert run.returncode == 0, run.stdout + run.stderr
