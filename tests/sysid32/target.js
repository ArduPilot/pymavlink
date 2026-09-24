const assert = require('assert');
const {mavlink20: mav, MAVLink20Processor: Link} = require(process.argv[2]);
for (const source of [42, 0xABCDEF12]) for (const target of [0, 7, 255, 256, 0xFFFFFFFF]) for (const signed of [false, true]) {
    const tx = new Link(null, source, 11);
    tx.signing.secret_key = Buffer.alloc(32, 42);
    tx.signing.sign_outgoing = signed;
    tx.signing.link_id = 3;
    tx.signing.timestamp = 1000;
    const message = new mav.messages.command_long(target, 250, 300, 1, 1, 2, 3, 4, 5, 6, 7);
    const bytes = message.pack(tx);
    console.log(Buffer.from(bytes).toString('hex'));
    const rx = new Link(null, 255, 1);
    if (signed) {
        rx.signing.secret_key = Buffer.alloc(32, 42);
        rx.signing.timestamp = 999;
    }
    const result = rx.parseBuffer(bytes);
    assert.strictEqual(result.length, 1);
    const decoded = result[0];
    assert.strictEqual(decoded._name, 'COMMAND_LONG');
    assert.strictEqual(decoded._header.srcSystem, source);
    assert.strictEqual(decoded.target_system, target);
    assert.strictEqual(decoded.target_component, 250);
    tx.signing.timestamp = 1000;
    assert.deepStrictEqual(decoded.pack(tx), bytes);
    message.target_system = 0;
    assert.strictEqual(message.pack(tx)[2] & 4, 0);
}
const tx = new Link(null, 42, 11);
const command = new mav.messages.command_long(0xABCDEF12, 250, 300, 1, 1, 2, 3, 4, 5, 6, 7);
command.pack(tx);
command.target_system = 7;
assert.strictEqual(command.pack(tx)[2] & 4, 0);
for (const target of [0, 7, 256, 0xFFFFFFFF]) {
    const original = new mav.messages.command_long(256, 250, 300, 1, 1, 2, 3, 4, 5, 6, 7);
    const received = new Link(null, 255, 1).parseBuffer(original.pack(tx))[0];
    received.target_system = target;
    received.target_component = 19;
    const forwarded = new Link(null, 255, 1).parseBuffer(received.pack(tx))[0];
    assert.strictEqual(forwarded.target_system, target);
    assert.strictEqual(forwarded.target_component, 19);
    assert.strictEqual(Boolean(forwarded._header.incompat_flags & 4), target > 255);
}
assert.strictEqual(typeof new mav.messages.heartbeat(2, 3, 81, 0, 4, 3).set_target, 'undefined');
