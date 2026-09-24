const assert = require('assert');
const fs = require('fs');
const lib = require(process.argv[2]);
for (const file of fs.readdirSync(process.argv[3]).filter(f => f.endsWith('.v2'))) {
    const bytes = fs.readFileSync(process.argv[3] + '/' + file);
    for (const fragmented of [false, true]) {
        const parser = new lib.MAVLink20Processor(null, 255, 1);
        let messages = [];
        for (const part of (fragmented ? Array.from(bytes, b => Buffer.from([b])) : [bytes])) {
            messages = messages.concat(parser.parseBuffer(part) || []);
        }
        assert.strictEqual(messages.length, 2, file);
        assert.strictEqual(messages[0].id, -1, file);
        assert.strictEqual(messages[1].name, 'HEARTBEAT', file);
        assert.strictEqual(messages[1].header.srcSystem, 42, file);
    }
}
