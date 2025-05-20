"""
Module to parse ArduPilot logs into pandas DataFrames with optional caching.

- Accepts a log file, list of messages/fields, and resample frequency.
- Aligns output rows to clean time intervals relative to the start of the log.
- Uses a lightweight caching mechanism keyed by file content and module hash.
- Automatically invalidates cache on module updates or parameter changes.

Intended for efficient, repeatable log analysis workflows.
"""

import os
import re
import json
import hashlib
import pandas as pd
from pymavlink.DFReader import DFReader_binary

HASH_SIZE_BYTES = 1024 * 1024  # 1MB


def parse_log_to_df(path, specs, frequency, cache_dir=None):
    manager = LogCacheManager(cache_dir)

    if manager:
        df = manager.try_load_dataframe(path, specs, frequency)
        if df is not None:
            return df

    reader = DFReader_binary(path)
    fields = expand_field_specs(specs, reader)
    # Dump the messages dict, so we get NaNs until the first valid message of each type
    reader.rewind()

    PERIOD = 1 / frequency
    first_timestamp = None
    next_emit_time = None

    rows = []
    # Make a list of base (non-instanced) message types
    filter_types = [re.sub(r"\[\d+\]", "", m) for m in fields.keys()]
    while True:
        msg = reader.recv_match(type=filter_types)
        if msg is None:
            break
        if not first_timestamp:
            first_timestamp = reader.clock.timestamp
            next_emit_time = first_timestamp
        if reader.clock.timestamp >= next_emit_time:
            n_period = (reader.clock.timestamp - first_timestamp) // PERIOD
            n_period += 1
            next_emit_time = first_timestamp + n_period * PERIOD
            rows.append(new_row(reader, fields))

    df = pd.DataFrame(rows)
    df["timestamp"] = pd.to_datetime(df["timestamp"], unit="s")
    df.set_index("timestamp", inplace=True)
    df = df[[f"{m}.{f}" for m in fields.keys() for f in fields[m]]]

    if manager:
        manager.save_dataframe(path, df, specs, frequency)

    return df


def new_row(reader: DFReader_binary, fields):
    row = {"timestamp": reader.clock.timestamp}
    for msg in fields.keys():
        if msg not in reader.messages:
            continue
        m = reader.messages[msg]
        for field in fields[msg]:
            row[f"{msg}.{field}"] = getattr(m, field, None)
    return row


def expand_field_specs(specs, reader: DFReader_binary):
    out = {}
    for spec in specs:
        msg, field = spec.split(".") if "." in spec else (spec, None)
        msg_base = re.sub(r"\[\d+\]", "", msg)
        if msg_base not in reader.name_to_id:
            raise ValueError(f"Message {msg_base} not found in log file")
        fmt = reader.formats[reader.name_to_id[msg_base]]
        if msg_base != msg and fmt.instance_field is None:
            raise ValueError(
                f"Message {msg_base} does not support instances, but {msg} was requested"
            )
        if field is not None:
            if field not in fmt.columns:
                raise ValueError(f"Field {field} not found in message {msg_base}")
            out.setdefault(msg, []).append(field)
        else:
            out.setdefault(msg, []).extend(fmt.columns)
    return out


class LogCacheManager:
    def __init__(self, cache_dir):
        self.cache_dir = cache_dir
        if cache_dir is not None:
            os.makedirs(cache_dir, exist_ok=True)

    def __bool__(self):
        return self.cache_dir is not None

    def _compute_key(self, path):
        stat = os.stat(path)
        size = stat.st_size
        with open(path, "rb") as f:
            data = f.read(HASH_SIZE_BYTES)
        h = hashlib.sha256(data).hexdigest()
        h = h[:16]  # 16 characters is plenty to prevent collisions
        return f"{h}_{size}"

    def _module_hash(self):
        with open(__file__, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()[:8]

    @staticmethod
    def _specs_equal(a, b):
        return set(a) == set(b)

    def try_load_dataframe(self, path, specs, frequency):
        key = self._compute_key(path)
        cache_path = os.path.join(self.cache_dir, f"{key}.feather")
        meta_path = os.path.join(self.cache_dir, f"{key}.meta.json")

        if os.path.exists(cache_path) and os.path.exists(meta_path):
            with open(meta_path, "r") as f:
                meta = json.load(f)
            if (
                self._specs_equal(meta.get("specs", []), specs)
                and meta.get("frequency") == frequency
                and meta.get("module_hash") == self._module_hash()
            ):
                return pd.read_feather(cache_path)
        return None

    def save_dataframe(self, path, df, specs, frequency):
        key = self._compute_key(path)
        cache_path = os.path.join(self.cache_dir, f"{key}.feather")
        meta_path = os.path.join(self.cache_dir, f"{key}.meta.json")

        df.reset_index(drop=True).to_feather(cache_path)
        meta = {
            "specs": specs,
            "frequency": frequency,
            "module_hash": self._module_hash(),
        }
        with open(meta_path, "w") as f:
            json.dump(meta, f)


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Parse a log file to a DataFrame.",
        usage="python -m pymavlink.dfreader_pandas <log_file> [options]",
        epilog="Example usage: python -m pymavlink.dfreader_pandas log.bin --fields TECS EFI CTUN.E2T BAT[0].Volt --frequency 10.0 --cache_dir ./.tmp",
    )
    parser.add_argument("path", type=str, help="Path to the log file")
    parser.add_argument(
        "--fields",
        type=str,
        nargs="+",
        default=[],
        help="Space-separated list of message types and fields to include in the DataFrame",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=10.0,
        help="Frequency in Hz for sampling the log file",
    )
    parser.add_argument(
        "--cache_dir",
        type=str,
        default=None,
        help="Directory to cache the DataFrame",
    )
    parser.add_argument(
        "--profile",
        action="store_true",
        help="Enable profiling of the parsing process",
    )
    args = parser.parse_args()

    if args.profile:
        from line_profiler import LineProfiler

        profiler = LineProfiler()
        profiler.add_function(parse_log_to_df)
        profiler.add_function(new_row)
        profiler.enable_by_count()

    if args.fields is None or len(args.fields) == 0:
        raise ValueError(
            "No fields provided. Use --fields to specify message types and/or fields."
        )

    df = parse_log_to_df(
        args.path, args.fields, args.frequency, args.cache_dir
    )
    print(df.head())
    print("...")
    print(df.tail())

    if args.profile:
        profiler.print_stats()


if __name__ == "__main__":
    main()
