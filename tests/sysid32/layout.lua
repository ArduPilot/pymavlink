local mav = require('mavlink_msgs')
for flags=0,7 do
  for _,storage in ipairs({16,256,264}) do
    local file = assert(io.open(ROOT..'/'..flags..'-'..storage..'.bin', 'rb'))
    local data = file:read('a')
    file:close()
    local msg = assert(mav.decode(data, {[0]='HEARTBEAT'}, true))
    assert(msg.sysid == (flags & 2 ~= 0 and 0xabcdef12 or 42))
    assert(msg.target_sysid == (flags & 4 ~= 0 and (flags & 2 ~= 0 and 0xfedcba98 or 7) or nil))
    assert(msg.base_mode == 81)
    if storage == 264 then assert(mav.decode(data, {[0]='HEARTBEAT'})) end
    local bad = data:sub(1,4)..string.char(flags | 128)..data:sub(6)
    assert(mav.decode(bad, {[0]='HEARTBEAT'}, true) == nil)
  end
end
