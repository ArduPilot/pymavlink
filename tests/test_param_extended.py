"""Exact bytewise parameter transport, including NaN patterns and uint64 limits."""
import io
from contextlib import redirect_stdout
import math
import os
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

from pymavlink import mavutil, mavparm
from pymavlink.generator import mavgen, mavparse

TEST_XML = '''<mavlink>
  <version>3</version>
  <enums><enum name="MAV_PARAM_TYPE">
      <description>Specifies the datatype of a MAVLink parameter.</description>
      <entry value="1" name="MAV_PARAM_TYPE_UINT8">
        <description>8-bit unsigned integer</description>
      </entry>
      <entry value="2" name="MAV_PARAM_TYPE_INT8">
        <description>8-bit signed integer</description>
      </entry>
      <entry value="3" name="MAV_PARAM_TYPE_UINT16">
        <description>16-bit unsigned integer</description>
      </entry>
      <entry value="4" name="MAV_PARAM_TYPE_INT16">
        <description>16-bit signed integer</description>
      </entry>
      <entry value="5" name="MAV_PARAM_TYPE_UINT32">
        <description>32-bit unsigned integer</description>
      </entry>
      <entry value="6" name="MAV_PARAM_TYPE_INT32">
        <description>32-bit signed integer</description>
      </entry>
      <entry value="7" name="MAV_PARAM_TYPE_UINT64">
        <description>64-bit unsigned integer</description>
      </entry>
      <entry value="8" name="MAV_PARAM_TYPE_INT64">
        <description>64-bit signed integer</description>
      </entry>
      <entry value="9" name="MAV_PARAM_TYPE_REAL32">
        <description>32-bit floating-point</description>
      </entry>
      <entry value="10" name="MAV_PARAM_TYPE_REAL64">
        <description>64-bit floating-point</description>
      </entry>
      <entry value="11" name="MAV_PARAM_TYPE_EXTENDED">
        <description>Value carried in the extended_type and extended_data fields of PARAM_VALUE and PARAM_SET. Only for use with components that advertise MAV_PROTOCOL_CAPABILITY_PARAM_BYTEWISE.</description>
      </entry>
      <entry value="12" name="MAV_PARAM_TYPE_BYTEWISE_INT32">
        <description>32-bit signed integer carried as four little-endian bytes in param_value, without floating-point conversion. The extension fields must be zero. Receivers must preserve the raw bytes, including NaN bit patterns.</description>
      </entry>
      <entry value="13" name="MAV_PARAM_TYPE_BYTEWISE_UINT32">
        <description>32-bit unsigned integer carried as four little-endian bytes in param_value, without floating-point conversion. The extension fields must be zero. Receivers must preserve the raw bytes, including NaN bit patterns.</description>
      </entry>
      <entry value="14" name="MAV_PARAM_TYPE_IN_PROGRESS">
        <description>PARAM_VALUE write-in-progress notification, only after the requester advertises MAV_PARAM_TYPES_SUPPORTED_IN_PROGRESS. Not a stored type and invalid in PARAM_SET. param_id identifies the pending write; receivers extend its timeout without updating parameter caches, types, counts or indices. Send param_value as NaN and extension fields as zero. Repeated identical PARAM_SET requests must not restart the operation. Completion is reported by PARAM_VALUE with the actual type/value, or PARAM_ERROR on failure.</description>
      </entry>
    </enum>
    <enum name="MAV_PARAM_EXTENDED_TYPE">
      <description>Specifies the datatype carried in the extended_data field of PARAM_VALUE and PARAM_SET when param_type is MAV_PARAM_TYPE_EXTENDED.</description>
      <entry value="0" name="MAV_PARAM_EXTENDED_TYPE_NONE">
        <description>No extended data. Used when param_type is not MAV_PARAM_TYPE_EXTENDED.</description>
      </entry>
      <entry value="1" name="MAV_PARAM_EXTENDED_TYPE_BYTEWISE_INT64">
        <description>64-bit signed integer, little-endian in the first 8 bytes of extended_data. Remaining bytes must be zero. param_value must be NaN and is ignored by receivers.</description>
      </entry>
      <entry value="2" name="MAV_PARAM_EXTENDED_TYPE_BYTEWISE_UINT64">
        <description>64-bit unsigned integer, little-endian in the first 8 bytes of extended_data. Remaining bytes must be zero. param_value must be NaN and is ignored by receivers.</description>
      </entry>
      <entry value="3" name="MAV_PARAM_EXTENDED_TYPE_BYTEWISE_REAL64">
        <description>IEEE-754 binary64 floating-point value, little-endian in the first 8 bytes of extended_data. Remaining bytes must be zero. param_value must be NaN and is ignored by receivers.</description>
      </entry>
      <entry value="4" name="MAV_PARAM_EXTENDED_TYPE_CUSTOM">
        <description>Opaque 128-byte value in extended_data, including any trailing zero bytes. Interpretation is defined by the parameter's metadata or component-specific schema, as for MAV_PARAM_EXT_TYPE_CUSTOM. Shorter application values must be zero-padded to 128 bytes; a variable-length format must encode its own length. param_value must be NaN and is ignored by receivers.</description>
      </entry>
    </enum>
    <enum name="MAV_PARAM_TYPES_SUPPORTED" bitmask="true">
      <description>Explicit parameter encodings understood by a requester. These bits are independent of MAV_PARAM_TYPE and MAV_PARAM_EXTENDED_TYPE numeric values. Zero, including an absent extension, requests legacy encoding. Unsupported bits are ignored. PARAM_VALUE is broadcast: when clients share a channel, use only encodings understood by all clients on that channel.</description>
      <entry value="1" name="MAV_PARAM_TYPES_SUPPORTED_BYTEWISE_INT32">
        <description>Understands MAV_PARAM_TYPE_BYTEWISE_INT32.</description>
      </entry>
      <entry value="2" name="MAV_PARAM_TYPES_SUPPORTED_BYTEWISE_UINT32">
        <description>Understands MAV_PARAM_TYPE_BYTEWISE_UINT32.</description>
      </entry>
      <entry value="4" name="MAV_PARAM_TYPES_SUPPORTED_BYTEWISE_INT64">
        <description>Understands MAV_PARAM_TYPE_EXTENDED with MAV_PARAM_EXTENDED_TYPE_BYTEWISE_INT64.</description>
      </entry>
      <entry value="8" name="MAV_PARAM_TYPES_SUPPORTED_BYTEWISE_UINT64">
        <description>Understands MAV_PARAM_TYPE_EXTENDED with MAV_PARAM_EXTENDED_TYPE_BYTEWISE_UINT64.</description>
      </entry>
      <entry value="16" name="MAV_PARAM_TYPES_SUPPORTED_BYTEWISE_REAL64">
        <description>Understands MAV_PARAM_TYPE_EXTENDED with MAV_PARAM_EXTENDED_TYPE_BYTEWISE_REAL64.</description>
      </entry>
      <entry value="32" name="MAV_PARAM_TYPES_SUPPORTED_CUSTOM">
        <description>Understands MAV_PARAM_TYPE_EXTENDED with MAV_PARAM_EXTENDED_TYPE_CUSTOM and preserves all 128 data bytes.</description>
      </entry>
      <entry value="64" name="MAV_PARAM_TYPES_SUPPORTED_IN_PROGRESS">
        <description>Understands MAV_PARAM_TYPE_IN_PROGRESS notifications and the asynchronous parameter-write state machine.</description>
      </entry>
    </enum>
    <enum name="MAV_PARAM_ERROR">
      <wip/>
      <!-- This enum is work-in-progress and it can therefore change. It should NOT be used in stable production environments. -->
      <description>Parameter protocol error types (see PARAM_ERROR).</description>
      <entry value="0" name="MAV_PARAM_ERROR_NO_ERROR">
        <description>No error occurred (not expected in PARAM_ERROR but may be used in future implementations.</description>
      </entry>
      <entry value="1" name="MAV_PARAM_ERROR_DOES_NOT_EXIST">
        <description>Parameter does not exist</description>
      </entry>
      <entry value="2" name="MAV_PARAM_ERROR_VALUE_OUT_OF_RANGE">
        <description>Parameter value does not fit within accepted range</description>
      </entry>
      <entry value="3" name="MAV_PARAM_ERROR_PERMISSION_DENIED">
        <description>Caller is not permitted to set the value of this parameter</description>
      </entry>
      <entry value="4" name="MAV_PARAM_ERROR_COMPONENT_NOT_FOUND">
        <description>Unknown component specified</description>
      </entry>
      <entry value="5" name="MAV_PARAM_ERROR_READ_ONLY">
        <description>Parameter is read-only</description>
      </entry>
      <entry value="6" name="MAV_PARAM_ERROR_TYPE_UNSUPPORTED">
        <description>Parameter data type (MAV_PARAM_TYPE) is not supported by flight stack (at all)</description>
      </entry>
      <entry value="7" name="MAV_PARAM_ERROR_TYPE_MISMATCH">
        <description>Parameter type does not match expected type</description>
      </entry>
      <entry value="9" name="MAV_PARAM_ERROR_WRITE_FAIL">
        <description>Parameter exists and its type/value are supported, but the write operation failed. Terminal result, including after MAV_PARAM_TYPE_IN_PROGRESS. The client may read the parameter again to obtain the current value.</description>
      </entry>
    </enum>
    </enums>
  <messages><message id="20" name="PARAM_REQUEST_READ">
      <description>Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -&gt; value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also https://mavlink.io/en/services/parameter.html for a full documentation of QGroundControl and IMU code.</description>
      <field type="uint8_t" name="target_system">System ID</field>
      <field type="uint8_t" name="target_component">Component ID</field>
      <field type="char[16]" name="param_id">Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string</field>
      <field type="int16_t" name="param_index" invalid="-1">Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)</field>
      <extensions/>
      <field type="uint32_t" name="supported_types" enum="MAV_PARAM_TYPES_SUPPORTED">Explicit encodings understood by the requester. Zero requests legacy encoding. The component caches this advertisement for subsequent parameter replies and updates on this channel; requests from other clients must not enable an encoding unsupported by an existing client.</field>
    </message>
    <message id="21" name="PARAM_REQUEST_LIST">
      <description>Request all parameters of this component. After this request, all parameters are emitted. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html</description>
      <field type="uint8_t" name="target_system">System ID</field>
      <field type="uint8_t" name="target_component">Component ID</field>
      <extensions/>
      <field type="uint32_t" name="supported_types" enum="MAV_PARAM_TYPES_SUPPORTED">Explicit encodings understood by the requester. Zero requests legacy encoding. The component caches this advertisement for subsequent parameter replies and updates on this channel; requests from other clients must not enable an encoding unsupported by an existing client.</field>
    </message>
    <message id="22" name="PARAM_VALUE">
      <description>Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html</description>
      <field type="char[16]" name="param_id">Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string</field>
      <field type="float" name="param_value">Onboard parameter value</field>
      <field type="uint8_t" name="param_type" enum="MAV_PARAM_TYPE">Onboard parameter type.</field>
      <field type="uint16_t" name="param_count">Total number of onboard parameters</field>
      <field type="uint16_t" name="param_index">Index of this onboard parameter</field>
      <extensions/>
      <field type="uint8_t" name="extended_type" enum="MAV_PARAM_EXTENDED_TYPE">Datatype of extended_data. Set (non-zero) only when param_type is MAV_PARAM_TYPE_EXTENDED, in which case param_value should be set to NaN.</field>
      <field type="uint8_t[128]" name="extended_data">Extended parameter value, encoded according to extended_type. Unused bytes must be zero; MAVLink2-truncated bytes are restored as zero. All 128 bytes are significant for CUSTOM.</field>
    </message>
    <message id="23" name="PARAM_SET">
      <description>Set a parameter value (write new value to permanent storage).
        The receiving component should acknowledge the new parameter value by broadcasting a PARAM_VALUE message (broadcasting ensures that multiple GCS all have an up-to-date list of all parameters). If the sending GCS did not receive a PARAM_VALUE within its timeout time, it should re-send the PARAM_SET message. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html.
      </description>
      <field type="uint8_t" name="target_system">System ID</field>
      <field type="uint8_t" name="target_component">Component ID</field>
      <field type="char[16]" name="param_id">Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string</field>
      <field type="float" name="param_value">Onboard parameter value</field>
      <field type="uint8_t" name="param_type" enum="MAV_PARAM_TYPE">Onboard parameter type.</field>
      <extensions/>
      <field type="uint8_t" name="extended_type" enum="MAV_PARAM_EXTENDED_TYPE">Datatype of extended_data. Set (non-zero) only when param_type is MAV_PARAM_TYPE_EXTENDED, in which case param_value should be set to NaN.</field>
      <field type="uint8_t[128]" name="extended_data">Extended parameter value, encoded according to extended_type. Unused bytes must be zero; MAVLink2-truncated bytes are restored as zero. All 128 bytes are significant for CUSTOM.</field>
    </message>
    <message id="345" name="PARAM_ERROR">
      <wip/>
      <!-- This enum is work-in-progress and it can therefore change. It should NOT be used in stable production environments. -->
      <description>Parameter set/get error. Returned from a MAVLink node in response to an error in the parameter protocol, for example failing to set a parameter because it does not exist.
      </description>
      <field type="uint8_t" name="target_system">System ID</field>
      <field type="uint8_t" name="target_component">Component ID</field>
      <field type="char[16]" name="param_id">Parameter id. Terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string</field>
      <field type="int16_t" name="param_index">Parameter index. Will be -1 if the param ID field should be used as an identifier (else the param id will be ignored)</field>
      <field type="uint8_t" name="error" enum="MAV_PARAM_ERROR">Error being returned to client.</field>
    </message>
    </messages>
</mavlink>
'''

def generate_dialect():
    '''generate a python module from the test XML, return the module'''
    tmpdir = tempfile.mkdtemp(prefix='param_extended_test')
    xml_path = os.path.join(tmpdir, 'param_extended.xml')
    with open(xml_path, 'w') as f:
        f.write(TEST_XML)
    out_path = os.path.join(tmpdir, 'param_extended_dialect.py')
    opts = mavgen.Opts(out_path, wire_protocol=mavparse.PROTOCOL_2_0, language='Python3')
    mavgen.mavgen(opts, [xml_path])
    sys.path.insert(0, tmpdir)
    try:
        import param_extended_dialect
    finally:
        sys.path.pop(0)
    return param_extended_dialect


class ParamBytewiseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dialect = generate_dialect()

    def pair(self, signed=False, wide=False):
        d = self.dialect
        tx = d.MAVLink(io.BytesIO(), srcSystem=0x12345678 if wide else 1, srcComponent=1)
        rx = d.MAVLink(io.BytesIO())
        if signed:
            for m in (tx, rx):
                m.signing.secret_key = bytes(range(32))
                m.signing.timestamp = 1
                m.signing.sign_outgoing = m is tx
        return tx, rx

    def message(self, value, ptype, subtype=0, set_message=False):
        d = self.dialect
        if set_message:
            msg = d.MAVLink_param_set_message(1, 1, b'TEST', 0.0, ptype)
        else:
            msg = d.MAVLink_param_value_message(b'TEST', 0.0, ptype, 1, 0)
        if ptype in (12, 13):
            msg.set_raw_field_bytes(0, struct.pack('<i' if ptype == 12 else '<I', value))
        else:
            msg.param_value = float('nan')
            msg.extended_type = subtype
            msg.extended_data = mavutil.encode_param_extended(value, subtype).ljust(128, b'\x00')
        return msg

    def test_exact_roundtrips(self):
        cases = [(12, 0, [-2**31, -2143289343, -1, 0, 4096, 16777217, 0x7f800001, 0x7fbfffff, 0x7fc00001, 2**31-1]),
                 (13, 0, [0, 0x7f800001, 0xff800001, 2**32-1]),
                 (11, 1, [-2**63, -1, 0, 2**53+1, 2**63-1]),
                 (11, 2, [0, 2**63+1, 2**64-1])]
        for ptype, subtype, values in cases:
            for value in values:
                for signed, wide, is_set in ((False, False, False), (True, False, True), (True, True, False)):
                    with self.subTest(ptype=ptype, value=value, signed=signed, wide=wide, is_set=is_set):
                        tx, rx = self.pair(signed, wide)
                        msg = self.message(value, ptype, subtype, is_set)
                        decoded = rx.parse_char(msg.pack(tx))
                        self.assertEqual(mavutil.decode_param_value(decoded), value)
                        self.assertEqual(decoded.get_srcSystem(), tx.srcSystem)
                        self.assertEqual(decoded.get_signed(), signed)
                        if ptype in (12, 13):
                            self.assertEqual(decoded.extended_type, 0)
                            self.assertFalse(any(decoded.extended_data))
                            self.assertEqual(decoded.get_msgbuf()[1], 23 if is_set else 25)
                        else:
                            self.assertTrue(math.isnan(decoded.param_value))
                        # Forwarding must retain sNaN-shaped integers too.
                        tx2, rx2 = self.pair()
                        self.assertEqual(mavutil.decode_param_value(rx2.parse_char(decoded.pack(tx2))), value)

    def test_raw_field_validation(self):
        msg = self.message(0x7f800001, 12)
        self.assertEqual(msg.get_raw_field_bytes(0, 4), struct.pack('<I', 0x7f800001))
        for offset, data in ((-1, b'x'), (msg.unpacker.size, b'x')):
            with self.assertRaises(ValueError):
                msg.set_raw_field_bytes(offset, data)
        msg.clear_raw_field_bytes()
        tx, rx = self.pair()
        self.assertEqual(mavutil.decode_param_value(rx.parse_char(msg.pack(tx))), 0)

    def test_unknown_extended_type(self):
        msg = self.message(42, 11, 1)
        msg.extended_type = 255
        self.assertTrue(math.isnan(mavutil.decode_param_value(msg)))

    def connection(self):
        tx, rx = self.pair()
        class Connection:
            target_system = 1
            target_component = 1
            param_fetch_start = 0
            mav = tx
            def mavlink20(self):
                return True
            def mavlink10(self):
                return True
        return Connection(), rx

    def test_request_negotiation(self):
        conn, rx = self.connection()
        with patch.object(mavutil, 'mavlink', self.dialect):
            mavutil.mavfile.param_fetch_one(conn, 'TEST')
            mavutil.mavfile.param_fetch_all(conn)
            mavutil.mavfile.param_fetch_one(conn, 'TEST', supported_types=0)
        msgs = rx.parse_buffer(conn.mav.file.getvalue())
        self.assertEqual([m.supported_types for m in msgs], [127, 127, 0])
        self.assertLessEqual(msgs[-1].get_msgbuf()[1], 20)

    def test_set_wrapper(self):
        conn, rx = self.connection()
        for value, ptype, subtype in ((0x7f800001, 12, 0), (2**32-1, 13, 0), (-2**63, 11, 1), (2**64-1, 11, 2)):
            kwargs = {'parm_raw': struct.pack('<I', value)} if ptype != 11 else {
                'extended_type': subtype, 'extended_data': struct.pack('<q' if subtype == 1 else '<Q', value)}
            mavutil.mavfile.param_set_send(conn, 'TEST', 0.0, parm_type=ptype, **kwargs)
        msgs = rx.parse_buffer(conn.mav.file.getvalue())
        self.assertEqual([mavutil.decode_param_value(m) for m in msgs], [0x7f800001, 2**32-1, -2**63, 2**64-1])
        self.assertEqual([m.get_seq() for m in msgs], list(range(4)))
        for kwargs in ({'parm_type': 12}, {'parm_type': 12, 'parm_raw': b'x'},
                       {'parm_type': 11, 'extended_type': 1, 'extended_data': bytes(4)}):
            with self.assertRaises(ValueError):
                mavutil.mavfile.param_set_send(conn, 'TEST', 0.0, **kwargs)
        conn.mavlink20 = lambda: False
        with self.assertRaises(ValueError):
            mavutil.mavfile.param_set_send(conn, 'TEST', 0.0, parm_type=11, extended_type=1, extended_data=bytes(8))

    def test_mavparm_exact_values(self):
        conn, rx = self.connection()
        conn.param_fetch_one = lambda name: None
        def send(name, value, **kwargs):
            conn.mav.file.seek(0)
            conn.mav.file.truncate()
            mavutil.mavfile.param_set_send(conn, name, value, **kwargs)
            msg = rx.parse_char(conn.mav.file.getvalue())
            decoded = mavutil.decode_param_value(msg)
            conn.ack = self.message(decoded, msg.param_type, msg.extended_type)
            conn.ack = rx.parse_char(conn.ack.pack(conn.mav))
        conn.param_set_send = send
        def receive(**kwargs):
            ack, conn.ack = conn.ack, None
            return ack
        conn.recv_match = receive
        params = mavparm.MAVParmDict()
        params.target_supports_bytewise = True
        for logical_type, value in ((6, 0x7f800001), (5, 2**32-1), (8, -2**63), (7, 2**64-1)):
            params.param_types['TEST'] = logical_type
            self.assertTrue(params.mavset(conn, 'TEST', str(value)))
            self.assertEqual(params['TEST'], value)
        for value in ('1.5', 'nan', str(2**64)):
            self.assertFalse(params.mavset(conn, 'TEST', value))
        with tempfile.NamedTemporaryFile() as f:
            params.save(f.name)
            restored = mavparm.MAVParmDict()
            self.assertTrue(restored.load(f.name))
            self.assertEqual(restored['TEST'], 2**64-1)
            restored['TEST'] = 2**64-2
            output = io.StringIO()
            with redirect_stdout(output):
                restored.diff(f.name)
            self.assertIn(str(2**64-1), output.getvalue())
            self.assertIn(str(2**64-2), output.getvalue())

    def test_real64_and_custom_wire(self):
        values = [(3, v) for v in (0.0, -0.0, 1.0000000000000002, 5e-324,
                                   sys.float_info.max, float('inf'), float('-inf'), float('nan'))]
        values += [(4, v) for v in (b'', b'hello\x00world', bytes(range(128)), bytes([255])*128)]
        for subtype, value in values:
            expected = mavutil.encode_param_extended(value, subtype).ljust(128, b'\x00')
            for is_set in (False, True):
                with self.subTest(subtype=subtype, value=value, is_set=is_set):
                    tx, rx = self.pair(signed=True, wide=True)
                    msg = self.message(value, 11, subtype, is_set)
                    decoded = rx.parse_char(msg.pack(tx))
                    self.assertTrue(decoded.get_signed())
                    self.assertEqual(bytes(decoded.extended_data), expected)
                    actual = mavutil.decode_param_value(decoded)
                    self.assertEqual(mavutil.encode_param_extended(actual, subtype).ljust(128, b'\x00'), expected)
                    if subtype == 4 and expected[-1]:
                        self.assertEqual(decoded.get_msgbuf()[1], 152 if is_set else 154)
        for value in (bytes(129), 'text requires explicit encoding'):
            with self.assertRaises(ValueError):
                mavutil.encode_param_extended(value, 4)
        conn, _ = self.connection()
        with self.assertRaises(ValueError):
            mavutil.mavfile.param_set_send(conn, 'TEST', 0, parm_type=14)
        conn, rx = self.connection()
        for subtype, value in ((3, 1.0000000000000002), (4, bytes(range(128)))):
            mavutil.mavfile.param_set_send(conn, 'TEST', 0, parm_type=11, extended_type=subtype,
                                         extended_data=mavutil.encode_param_extended(value, subtype))
        self.assertEqual([mavutil.decode_param_value(m) for m in rx.parse_buffer(conn.mav.file.getvalue())],
                         [1.0000000000000002, bytes(range(128))])

    def response(self, msg, system=1, component=1):
        tx = self.dialect.MAVLink(io.BytesIO(), srcSystem=system, srcComponent=component)
        rx = self.dialect.MAVLink(io.BytesIO())
        return rx.parse_char(msg.pack(tx))

    def test_progress_lifecycle(self):
        # A deterministic clock exercises a write longer than the normal timeout.
        for fail in (False, True):
            conn, _ = self.connection()
            conn.param_fetch_one = lambda name: None
            sends = []
            conn.param_set_send = lambda *args, **kw: sends.append((args, kw))
            clock = [10.0]
            progress = self.dialect.MAVLink_param_value_message(b'TEST', float('nan'), 14, 999, 999)
            final = (self.dialect.MAVLink_param_error_message(1, 1, b'TEST', -1, 9) if fail else
                     self.message(1.0000000000000002, 11, 3))
            replies = [self.response(progress) for _ in range(5)] + [self.response(final)]
            def receive(**kwargs):
                clock[0] += 0.7
                return replies.pop(0)
            conn.recv_match = receive
            params = mavparm.MAVParmDict()
            params['TEST'] = 42
            with patch.object(mavparm.time, 'time', side_effect=lambda: clock[0]):
                self.assertEqual(params.mavset(conn, 'TEST', 1.0000000000000002,
                                               parm_type=11, extended_type=3), not fail)
            self.assertEqual(len(sends), 1)
            self.assertEqual(params['TEST'], 42 if fail else 1.0000000000000002)

    def test_response_identity_and_cache(self):
        conn, _ = self.connection()
        msg = self.message(42, 12)
        self.assertTrue(mavutil.param_response_matches(self.response(msg), conn, 'TEST'))
        self.assertFalse(mavutil.param_response_matches(self.response(msg, system=2), conn, 'TEST'))
        self.assertFalse(mavutil.param_response_matches(self.response(msg, component=2), conn, 'TEST'))
        error = self.dialect.MAVLink_param_error_message(2, 1, b'TEST', -1, 9)
        self.assertFalse(mavutil.param_response_matches(self.response(error), conn, 'TEST'))
        connection = mavutil.mavfile(None, 'test', input=False)
        connection.target_system, connection.target_component = 1, 1
        connection.post_message(self.response(msg))
        progress = self.dialect.MAVLink_param_value_message(b'TEST', float('nan'), 14, 999, 999)
        connection.post_message(self.response(progress))
        self.assertEqual(connection.params['TEST'], 42)
        self.assertIsNone(mavutil.decode_param_value(progress))

    def test_progress_stops_then_times_out(self):
        conn, _ = self.connection()
        conn.param_fetch_one = lambda name: None
        sends = []
        conn.param_set_send = lambda *args, **kw: sends.append(kw)
        clock = [0.0]
        progress = self.response(self.dialect.MAVLink_param_value_message(b'TEST', float('nan'), 14, 0, 0))
        replies = [progress]
        conn.recv_match = lambda **kw: replies.pop() if replies else None
        def sleep(delay):
            clock[0] += delay
        with patch.object(mavparm.time, 'time', side_effect=lambda: clock[0]), patch.object(mavparm.time, 'sleep', sleep):
            self.assertFalse(mavparm.MAVParmDict().mavset(conn, 'TEST', 42, retries=2))
        self.assertEqual(len(sends), 2)
        self.assertLess(clock[0], 3)

    def test_custom_ack_and_files(self):
        conn, _ = self.connection()
        conn.param_fetch_one = lambda name: None
        value = bytes(range(128))
        conn.param_set_send = lambda *args, **kw: None
        # Wrong padding must not complete the write.
        replies = [self.response(self.message(value[:-1], 11, 4)), self.response(self.message(value, 11, 4))]
        conn.recv_match = lambda **kw: replies.pop(0)
        params = mavparm.MAVParmDict()
        self.assertTrue(params.mavset(conn, 'TEST', value, parm_type=11, extended_type=4))
        self.assertEqual(params['TEST'], value)
        params['DOUBLE'] = 1.0000000000000002
        params['ZERO'] = -0.0
        params.param_types['DOUBLE'] = 10
        with tempfile.NamedTemporaryFile() as f:
            params.save(f.name)
            restored = mavparm.MAVParmDict()
            self.assertTrue(restored.load(f.name))
            self.assertEqual(restored, params)
            self.assertEqual(struct.pack('<d', restored['ZERO']), struct.pack('<d', -0.0))
            restored.save(f.name)
            again = mavparm.MAVParmDict()
            again.load(f.name)
            self.assertEqual(again, params)
            restored['TEST'] = bytes(128)
            with redirect_stdout(io.StringIO()):
                restored.diff(f.name)

if __name__ == '__main__':
    unittest.main()
