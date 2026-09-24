import {MAVLinkModule as BaseModule, MAVLinkMessage} from '@ifrunistuttgart/node-mavlink';

/** Stream framing guard for the external runtime, which supports neither
 * extended headers nor signatures. Import this class from message-registry
 * instead of importing MAVLinkModule directly from node-mavlink.
 */
export class MAVLinkModule extends BaseModule {
    private pending: Buffer = Buffer.alloc(0);
    public unsupportedFrames = 0;

    public parse(bytes: Buffer): Promise<MAVLinkMessage[]> {
        const combined = Buffer.alloc(this.pending.length + bytes.length);
        combined.set(this.pending);
        combined.set(bytes, this.pending.length);
        this.pending = combined;
        const results: Array<Promise<MAVLinkMessage[]>> = [];
        while (this.pending.length > 0) {
            const magic = this.pending[0];
            if (magic !== 0xfd && magic !== 0xfe) {
                this.pending = this.pending.slice(1);
                continue;
            }
            if (this.pending.length < 3) break;
            const flags = magic === 0xfd ? this.pending[2] : 0;
            let length = this.pending[1] + (magic === 0xfd ? 12 : 8);
            if (flags & 1) length += 13;
            if (flags & 2) length += 3;
            if (flags & 4) length += 4;
            if (this.pending.length < length) break;
            const frame = this.pending.slice(0, length);
            this.pending = this.pending.slice(length);
            if (flags !== 0) {
                this.unsupportedFrames++;
                continue;
            }
            results.push(super.parse(frame));
        }
        return Promise.all(results).then(batches => ([] as MAVLinkMessage[]).concat(...batches));
    }
}
