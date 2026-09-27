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
      <entry value="12" name="MAV_PARAM_TYPE_BYTEWISE_INT32">
        <description>32-bit signed integer carried as four little-endian bytes in param_value, without floating-point conversion. The extension fields must be zero. Receivers must preserve the raw bytes, including NaN bit patterns.</description>
      </entry>
      <entry value="13" name="MAV_PARAM_TYPE_BYTEWISE_UINT32">
        <description>32-bit unsigned integer carried as four little-endian bytes in param_value, without floating-point conversion. The extension fields must be zero. Receivers must preserve the raw bytes, including NaN bit patterns.</description>
      </entry>
      <entry value="11" name="MAV_PARAM_TYPE_EXTENDED">
        <description>Value carried in the extended_type and extended_data fields of PARAM_VALUE and PARAM_SET. Only for use with components that advertise MAV_PROTOCOL_CAPABILITY_PARAM_BYTEWISE.</description>
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
    </enum>
    </enums><messages><message id="20" name="PARAM_REQUEST_READ">
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
      <field type="uint8_t[32]" name="extended_data">Extended parameter value, encoded according to extended_type.</field>
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
      <field type="uint8_t[32]" name="extended_data">Extended parameter value, encoded according to extended_type.</field>
    </message>
    </messages></mavlink>
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
            msg.extended_data = struct.pack('<q' if subtype == 1 else '<Q', value) + bytes(24)
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
        self.assertEqual([m.supported_types for m in msgs], [15, 15, 0])
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
            conn.ack.param_id = name
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

if __name__ == '__main__':
    unittest.main()
