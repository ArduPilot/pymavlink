"""Run the generated Lua mavlink_msgs.encode() on messages with unset fields."""
from tests.test_sysid32_generators import generate, lua_run


def test_lua_encode_unset_fields(tmp_path):
    """An extension field the caller did not set (e.g. one added after the
    script was written) is encoded as zero, an empty string for a char
    array, or all zeros for an array.  Any other unset field raises an
    error naming it, so a misspelt field is not silently sent as zero.
    """
    modules = generate(tmp_path / 'modules', 'Lua')
    lua_run("package.path = %r .. '/?.lua;' .. package.path\n" % str(modules) + """
local mav = require('mavlink_msgs')

local function encode_error(msgname, message)
  local ok, err = pcall(mav.encode, msgname, message)
  assert(not ok, msgname .. ' encoded with a missing field')
  return err
end

-- extension fields progress, result_param2, target_system, target_component
local id, payload = mav.encode('COMMAND_ACK', {command = 300, result = 4})
assert(id == 77)
assert(payload == string.pack('<HBBi4BB', 300, 4, 0, 0, 0, 0))

-- extension char[200] tune2
id, payload = mav.encode('PLAY_TUNE', {target_system = 1, target_component = 2, tune = 'MFT'})
assert(id == 258)
assert(payload == string.pack('<BBc30c200', 1, 2, 'MFT', ''))

-- extension float[4] repr_offset_q
id, payload = mav.encode('ATTITUDE_QUATERNION', {time_boot_ms = 1, q1 = 1, q2 = 0, q3 = 0, q4 = 0,
                                                 rollspeed = 0, pitchspeed = 0, yawspeed = 0})
assert(id == 31)
assert(payload == string.pack('<I4fffffffffff', 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

-- base fields: misspelt uint8_t target_system, char[16] param_id, uint8_t[70] data
local err = encode_error('COMMAND_LONG', {target_sytem = 1, target_component = 1, command = 400, confirmation = 0,
                                          param1 = 1, param2 = 0, param3 = 0, param4 = 0, param5 = 0,
                                          param6 = 0, param7 = 0})
assert(string.find(err, 'target_system', 1, true), err)
err = encode_error('PARAM_SET', {target_system = 1, target_component = 1, param_value = 1.5, param_type = 9})
assert(string.find(err, 'param_id', 1, true), err)
err = encode_error('SERIAL_CONTROL', {device = 0, flags = 0, timeout = 0, baudrate = 0, count = 0})
assert(string.find(err, 'data', 1, true), err)
""")
