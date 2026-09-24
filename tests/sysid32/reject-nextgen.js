const assert = require('assert');
const fs = require('fs');
const {MAVLink20Processor: Link} = require(process.argv[2]);
const directory = process.argv[3];
for (const file of fs.readdirSync(directory).filter(f => /^(128|129)-.*\.v2$/.test(f))) {
    const bytes = fs.readFileSync(directory + '/' + file);
    const rejected = fs.readFileSync(directory + '/' + file.replace('.v2', '.frame'));
    for (const fragmented of [false, true]) {
        const parser = new Link(null, 255, 1);
        parser.signing.secret_key = Buffer.alloc(32, 42);
        parser.signing.timestamp = 999;
        parser.signing.allow_unsigned_callback = () => true;
        assert.throws(() => parser.decode(rejected), /Unsupported incompat_flags/, file);
        let messages = [];
        for (const part of (fragmented ? Array.from(bytes, b => Buffer.from([b])) : [bytes])) {
            messages = messages.concat(parser.parseBuffer(part) || []);
        }
        assert.strictEqual(messages.length, 2, file);
        assert.strictEqual(messages[0]._id, -1, file);
        assert.match(messages[0]._reason, /Unsupported incompat_flags/, file);
        assert.strictEqual(messages[1]._name, 'HEARTBEAT', file);
        assert.strictEqual(messages[1]._header.srcSystem, 42, file);
    }
}
