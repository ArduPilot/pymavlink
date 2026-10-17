import {MAVLinkModule as BaseModule, MAVLinkMessage,
        readInt64LE, readUInt64LE} from '@ifrunistuttgart/node-mavlink';
import {writeInt64LE, writeUInt64LE} from '@ifrunistuttgart/node-mavlink/lib/mavlink-message';
import {createHash, timingSafeEqual} from 'crypto';

type Factory = new (system: number, component: number) => MAVLinkMessage;

/** Import this module from message-registry to use extended system IDs.
 * Payload structs and their metadata remain compatible with node-mavlink.
 */
export class MAVLinkModule extends BaseModule {
    private pending: Buffer = Buffer.alloc(0);
    declare private factories: Map<number, Factory>;
    private sequence = 0;
    public unsupportedFrames = 0;
    public badSignatures = 0;
    public signing = {secretKey: null as Buffer | null, signOutgoing: false,
                      linkId: 0, timestamp: 0, streams: new Map<string, number>()};

    public registerMessages(registry: Array<[number, Factory]>): void {
        super.registerMessages(registry);
        if (!this.factories) this.factories = new Map();
        for (const [id, factory] of registry) this.factories.set(id, factory);
    }

    // node-mavlink bundles older Node type declarations. set() also works
    // when its Buffer declarations differ from the application's Node types.
    private join(parts: Buffer[]): Buffer {
        const result = Buffer.alloc(parts.reduce((n, p) => n + p.length, 0));
        let offset = 0;
        for (const part of parts) { result.set(part, offset); offset += part.length; }
        return result;
    }

    private crc(bytes: Buffer, extra: number): number {
        let crc = 0xffff;
        for (const value of [...bytes, extra]) {
            let tmp = value ^ (crc & 255);
            tmp = (tmp ^ (tmp << 4)) & 255;
            crc = ((crc >>> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >>> 4)) & 0xffff;
        }
        return crc;
    }

    private payload(message: MAVLinkMessage, input?: Buffer, v2 = true): Buffer {
        const fields = message._message_fields.filter(f => v2 || !f[2]);
        const arrays = message._array_lengths || {};
        const length = fields.reduce((n, f) => n + message.sizeof(f[1]) * (arrays[f[0]] || 1), 0);
        const bytes = Buffer.alloc(length);
        if (input) bytes.set(input.slice(0, length));
        let offset = 0;
        for (const [name, type] of fields) {
            const count = arrays[name] || 1;
            if (type === 'char') {
                if (input) message[name] = bytes.toString('ascii', offset, offset + count).split('\0')[0];
                else bytes.write(message[name] || '', offset, count, 'ascii');
                offset += count;
                continue;
            }
            const values: number[] = [];
            for (let i = 0; i < count; i++) {
                const size = message.sizeof(type);
                let value = (arrays[name] ? (message[name] ?? [])[i] : message[name]) ?? 0;
                if (name === message._target_system_field && value > 255) value = 255;
                if (input) {
                    if (type === 'float') value = bytes.readFloatLE(offset);
                    else if (type === 'double') value = bytes.readDoubleLE(offset);
                    else if (type === 'uint64_t') value = readUInt64LE(bytes, offset);
                    else if (type === 'int64_t') value = readInt64LE(bytes, offset);
                    else value = type.startsWith('uint') ? bytes.readUIntLE(offset, size) : bytes.readIntLE(offset, size);
                    values.push(value);
                } else {
                    if (type === 'float') bytes.writeFloatLE(value, offset);
                    else if (type === 'double') bytes.writeDoubleLE(value, offset);
                    else if (type === 'uint64_t') writeUInt64LE(bytes, value, offset);
                    else if (type === 'int64_t') writeInt64LE(bytes, value, offset);
                    else if (type.startsWith('uint')) bytes.writeUIntLE(value, offset, size);
                    else bytes.writeIntLE(value, offset, size);
                }
                offset += size;
            }
            if (input) message[name] = arrays[name] ? values : values[0];
        }
        return bytes;
    }

    public pack(messages: MAVLinkMessage[]): Buffer {
        return this.join(messages.map(message => {
            const v2 = this.protocol_version === 2;
            const source = message._system_id;
            const target = message._target_system_field ? (message[message._target_system_field] ?? 0) : 0;
            for (const id of [source, target]) {
                if (!Number.isInteger(id) || id < 0 || id > 0xffffffff) throw new RangeError('System ID must be uint32');
            }
            if (!v2 && (source > 255 || target > 255 || message._message_id > 255)) throw new Error('MAVLink 1 cannot encode extended IDs');
            let payload = this.payload(message, undefined, v2);
            if (v2) {
                let length = payload.length;
                while (length > 1 && payload[length - 1] === 0) length--;
                payload = payload.slice(0, length);
            }
            const signed = v2 && this.signing.signOutgoing;
            const flags = (source > 255 ? 2 : 0) | (target > 255 ? 4 : 0) | (signed ? 1 : 0);
            const headerLength = v2 ? 10 + (flags & 2 ? 3 : 0) + (flags & 4 ? 4 : 0) : 6;
            const bytes = Buffer.alloc(headerLength + payload.length + 2 + (signed ? 13 : 0));
            bytes[0] = v2 ? 0xfd : 0xfe;
            bytes[1] = payload.length;
            if (v2) bytes[2] = flags;
            let offset = v2 ? 4 : 2;
            bytes[offset++] = this.sequence;
            this.sequence = (this.sequence + 1) & 255;
            const sourceLength = flags & 2 ? 4 : 1;
            bytes.writeUIntLE(source, offset, sourceLength); offset += sourceLength;
            bytes.writeUInt8(message._component_id, offset++);
            bytes.writeUIntLE(message._message_id, offset, v2 ? 3 : 1); offset += v2 ? 3 : 1;
            if (flags & 4) bytes.writeUInt32LE(target, offset);
            bytes.set(payload, headerLength);
            const crcOffset = headerLength + payload.length;
            bytes.writeUInt16LE(this.crc(bytes.slice(1, crcOffset), message._crc_extra), crcOffset);
            if (signed) {
                if (!this.signing.secretKey || this.signing.secretKey.length !== 32) throw new Error('Signing key must have 32 bytes');
                bytes.writeUInt8(this.signing.linkId, crcOffset + 2);
                bytes.writeUIntLE(this.signing.timestamp, crcOffset + 3, 6);
                bytes.set(createHash('sha256').update(this.signing.secretKey).update(bytes.slice(0, crcOffset + 9)).digest().slice(0, 6), crcOffset + 9);
                this.signing.timestamp++;
            }
            return bytes;
        }));
    }

    public parse(bytes: Buffer): Promise<MAVLinkMessage[]> {
        this.pending = this.join([this.pending, bytes]);
        const results: MAVLinkMessage[] = [];
        while (this.pending.length > 0) {
            const magic = this.pending[0];
            if (magic !== 0xfd && magic !== 0xfe) { this.pending = this.pending.slice(1); continue; }
            if (this.pending.length < 3) break;
            const v2 = magic === 0xfd;
            const flags = v2 ? this.pending[2] : 0;
            const headerLength = v2 ? 10 + (flags & 2 ? 3 : 0) + (flags & 4 ? 4 : 0) : 6;
            const crcOffset = headerLength + this.pending[1];
            const length = crcOffset + 2 + (flags & 1 ? 13 : 0);
            if (this.pending.length < length) break;
            const frame = this.pending.slice(0, length);
            this.pending = this.pending.slice(length);
            if (flags & ~7) { this.unsupportedFrames++; continue; }
            let offset = v2 ? 5 : 3;
            const sourceLength = flags & 2 ? 4 : 1;
            const source = frame.readUIntLE(offset, sourceLength); offset += sourceLength;
            const component = frame[offset++];
            const id = frame.readUIntLE(offset, v2 ? 3 : 1); offset += v2 ? 3 : 1;
            const factory = this.factories.get(id);
            if (!factory) continue;
            const message = new factory(source, component);
            if (this.crc(frame.slice(1, crcOffset), message._crc_extra) !== frame.readUInt16LE(crcOffset)) continue;
            if (flags & 1) {
                const key = this.signing.secretKey;
                if (!key || key.length !== 32) { this.badSignatures++; continue; }
                const tag = createHash('sha256').update(key).update(frame.slice(0, crcOffset + 9)).digest().slice(0, 6);
                const timestamp = frame.readUIntLE(crcOffset + 3, 6);
                const stream = source + ':' + component + ':' + frame[crcOffset + 2];
                const previous = this.signing.streams.get(stream);
                if (!timingSafeEqual(tag, frame.slice(crcOffset + 9)) ||
                    (previous !== undefined && timestamp <= previous) ||
                    (previous === undefined && timestamp + 6000000 < this.signing.timestamp)) { this.badSignatures++; continue; }
                this.signing.streams.set(stream, timestamp);
                this.signing.timestamp = Math.max(timestamp, this.signing.timestamp);
            }
            this.payload(message, frame.slice(headerLength, crcOffset));
            if (flags & 4 && message._target_system_field) message[message._target_system_field] = frame.readUInt32LE(offset);
            message._sequence = frame[v2 ? 4 : 2];
            message._incompat_flags = flags;
            results.push(message);
            this.emit(message._message_name, message);
            this.emit('message', message);
        }
        return Promise.resolve(results);
    }
}
