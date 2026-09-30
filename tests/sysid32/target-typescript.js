const assert = require('assert');
const {MAVLinkModule: Link, messageRegistry} = require(process.argv[2]);
function link() { const l = new Link(messageRegistry, 42, false); l.upgradeLink(); return l; }
const Command = messageRegistry.find(([id])=>id===76)[1];
(async function() {
for (const source of [42, 0xABCDEF12]) for (const target of [0,7,255,256,0xffffffff]) for (const signed of [false,true]) {
    const tx = link(), rx = link();
    tx.signing.secretKey = Buffer.alloc(32,42); tx.signing.signOutgoing = signed;
    tx.signing.timestamp = 1000; tx.signing.linkId = 3;
    rx.signing.secretKey = tx.signing.secretKey;
    const message = new Command(source, 11);
    Object.assign(message,{target_system:target,target_component:250,command:300,confirmation:1,param1:1,param2:2,param3:3,param4:4,param5:5,param6:6,param7:7});
    const bytes = tx.pack([message]);
    console.log(bytes.toString('hex'));
    let result = [];
    for (const byte of bytes) result.push(...await rx.parse(Buffer.from([byte])));
    assert.strictEqual(result.length,1);
    const decoded = result[0];
    assert.strictEqual(decoded._system_id,source); assert.strictEqual(decoded.target_system,target);
    assert.strictEqual(decoded.param7,7); assert.strictEqual(decoded.target_component,250);
    if (signed) {
        assert.deepStrictEqual(await rx.parse(bytes),[]);
        const tampered = Buffer.from(bytes); tampered[tampered.length-1]^=1;
        const bad=link();bad.signing.secretKey=tx.signing.secretKey;
        assert.deepStrictEqual(await bad.parse(tampered),[]);
    }
    decoded.target_system=7;
    assert.strictEqual(tx.pack([decoded])[2]&4,0);
    tx.downgradeLink(); decoded.target_system=256;
    assert.throws(()=>tx.pack([decoded]));
}
})().catch(e=>{console.error(e);process.exit(1)});
