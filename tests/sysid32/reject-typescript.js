const assert = require('assert');
const fs = require('fs');
const {MAVLinkModule, messageRegistry} = require(process.argv[2]);
(async function() {
    for (const file of fs.readdirSync(process.argv[3]).filter(f => f.endsWith('.v2'))) {
        const bytes = fs.readFileSync(process.argv[3] + '/' + file);
        for (const fragmented of [false, true]) {
            const link = new MAVLinkModule(messageRegistry, 255, false);
            link.upgradeLink();
            link.on('error', error => { throw error; });
            let emitted = 0;
            link.on('message', message => { assert.strictEqual(message._message_id, 0); emitted++; });
            let messages = [];
            for (const part of (fragmented ? Array.from(bytes, b => Buffer.from([b])) : [bytes])) {
                messages = messages.concat(await link.parse(part));
            }
            assert.strictEqual(link.unsupportedFrames, 1, file);
            assert.strictEqual(messages.length, 1, file);
            assert.strictEqual(emitted, 1, file);
            assert.strictEqual(messages[0]._system_id, 42, file);
        }
    }
})().catch(error => { console.error(error); process.exit(1); });
