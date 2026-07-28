"""
Module to parse ArduPilot logs into pandas DataFrames with optional caching.

- Accepts a log file, list of messages/fields, and resample frequency.
- Extracts both flight data and parameter changes as separate DataFrames.
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


def load_log(path, fields, frequency, cache_dir=None):
    """Load data and parameters from a log file."""
    manager = LogCacheManager(cache_dir)
    manager.try_load(path, fields, frequency)

    if manager.data is not None and manager.params is not None:
        return manager.data, manager.params

    reader = DFReader_binary(path)
    if manager.data is None:
        manager.data = parse_log_data(reader, fields, frequency)
    if manager.params is None:
        manager.params = parse_log_params(reader)
    manager.save()
    return manager.data, manager.params

def load_log_params(path, cache_dir=None):
    """Load only parameters from a log file."""
    manager = LogCacheManager(cache_dir)
    manager.try_load(path)

    if manager.params is not None:
        return manager.params

    reader = DFReader_binary(path)
    manager.params = parse_log_params(reader)
    manager.save()
    return manager.params

def load_log_data(path, fields, frequency, cache_dir=None):
    """Load only data from a log file."""
    manager = LogCacheManager(cache_dir)
    manager.try_load(path, fields, frequency)

    if manager.data is not None:
        return manager.data

    reader = DFReader_binary(path)
    manager.data = parse_log_data(reader, fields, frequency)
    manager.save()
    return manager.data

def parse_log_data(reader: DFReader_binary, fields, frequency):
    fields = expand_field_specs(fields, reader)
    # Ensures missing data is NaN until first message of each type is received
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
    df["timestamp"] = pd.to_datetime(df["timestamp"], unit="s", utc=True)
    df.set_index("timestamp", inplace=True)
    df = df[[f"{m}.{f}" for m in fields.keys() for f in fields[m]]]

    return df


def parse_log_params(reader):
    """Parse parameters from the log file."""
    param_dict = {}
    reader.rewind()

    while True:
        msg = reader.recv_match(type="PARM")
        if msg is None:
            break
        name = msg.Name
        ts = msg._timestamp
        val = msg.Value
        default = msg.Default

        entry = param_dict.setdefault(name, [])

        # A NaN default means "no change"
        if pd.isna(default) and len(entry) > 0:
            default = entry[-1][2]

        # We force the first timestamp to be time-of-boot. This allows users
        # to easily use asof() on the DataFrame later (since they don't have to
        # worry about using too early of a timestamp).
        if not entry:
            ts = reader.clock.timebase

        # Log every change in value/default
        if not entry or entry[-1][0] != val or entry[-1][1] != default:
            entry.append((ts, val, default))


    # Flatten the dictionary list of index/value tuples
    index = []
    values = []
    for name, entries in param_dict.items():
        for entry in entries:
            index.append((name, pd.to_datetime(entry[0], unit="s", utc=True)))
            values.append((entry[1], entry[2]))
    # Create a DataFrame with a multi-index
    df = pd.DataFrame(values, index=pd.MultiIndex.from_tuples(index))
    df.columns = ["Value", "Default"]
    df.index.names = ["Name", "Timestamp"]

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
    """Cache manager for log files."""

    _module_hash = None

    def __init__(self, cache_dir):
        self.init_module_hash()
        self.cache_dir = cache_dir
        self.data = None
        self.params = None
        if cache_dir is not None:
            os.makedirs(cache_dir, exist_ok=True)

    @classmethod
    def init_module_hash(cls):
        """Initialize the module hash for the current module."""
        if cls._module_hash is None:
            with open(__file__, "rb") as f:
                cls._module_hash = hashlib.sha256(f.read()).hexdigest()[:8]

    @staticmethod
    def _compute_key(path):
        """Compute a unique key for a log file based on content and size."""
        if not os.path.exists(path):
            raise FileNotFoundError(f"File {path} does not exist")
        stat = os.stat(path)
        size = stat.st_size
        with open(path, "rb") as f:
            data = f.read(HASH_SIZE_BYTES)
        h = hashlib.sha256(data).hexdigest()
        h = h[:16]  # 16 characters is plenty to prevent collisions
        return f"{h}_{size}"

    def _data_cache_path(self):
        """Compute the cache path for a given key."""
        if self.cache_dir is None:
            return None
        return os.path.join(self.cache_dir, f"{self._key}.feather")

    def _param_cache_path(self):
        """Compute the parameter cache path for a given key."""
        if self.cache_dir is None:
            return None
        return os.path.join(self.cache_dir, f"{self._key}.params.feather")

    def _data_meta_path(self):
        """Compute the metadata path for a given key."""
        if self.cache_dir is None:
            return None
        return os.path.join(self.cache_dir, f"{self._key}.meta.json")

    def _param_meta_path(self):
        """Compute the parameter metadata path for a given key."""
        if self.cache_dir is None:
            return None
        return os.path.join(self.cache_dir, f"{self._key}.params.meta.json")

    @classmethod
    def _data_meta(cls, fields, frequency):
        """Generate metadata for the cache file."""
        return {
            "fields": fields,
            "frequency": frequency,
            "module_hash": cls._module_hash,
        }

    @classmethod
    def _param_meta(cls):
        """Generate metadata for the parameter cache file."""
        return {
            "module_hash": cls._module_hash,
        }

    @staticmethod
    def _compare_metadata(meta1, meta2):
        """Compare metadata for equality."""
        if meta1 is None or meta2 is None:
            return False
        if meta1.keys() != meta2.keys():
            return False
        for k in meta1.keys():
            if meta1[k] != meta2[k]:
                return False
        return True

    def try_load(self, path, fields=None, frequency=None):
        """Try to load a cached data and params for this log file.

        If we store the information needed to save the cache file later in
        case a cache file does not exist yet.
        """
        if self.cache_dir is None or not os.path.exists(self.cache_dir):
            return None, None
        self._log_path = path
        self._key = self._compute_key(path)
        self._data_meta = self._data_meta(fields, frequency)
        self._param_meta = self._param_meta()
        data_cache_path = self._data_cache_path()
        data_meta_path = self._data_meta_path()

        if os.path.exists(data_cache_path) and os.path.exists(data_meta_path):
            with open(data_meta_path, "r") as f:
                meta = json.load(f)
            if self._compare_metadata(meta, self._data_meta):
                self.data = pd.read_feather(data_cache_path)

        param_cache_path = self._param_cache_path()
        param_meta_path = self._param_meta_path()
        if os.path.exists(param_cache_path) and os.path.exists(
            param_meta_path
        ):
            with open(param_meta_path, "r") as f:
                meta = json.load(f)
            if self._compare_metadata(meta, self._param_meta):
                self.params = pd.read_feather(param_cache_path)

        return self.data, self.params

    def save(self):
        """Save the DataFrame to a cache file."""
        if self.cache_dir is None or not os.path.exists(self.cache_dir):
            return
        if self.data is None and self.params is None:
            raise ValueError(
                "Either data or params must be provided to save the cache"
            )
        key = self._compute_key(self._log_path)
        if self._key != key:
            print(
                f"Warning: cache key {self._key} does not match computed key "
                f"{key}. This suggests the log file has changed since it was "
                "opened. Your data is likely truncated; it will not be saved "
                "to the cache."
            )
            return

        if self.data is not None:
            self.data.to_feather(self._data_cache_path())
            with open(self._data_meta_path(), "w") as f:
                json.dump(self._data_meta, f)

        if self.params is not None:
            self.params.to_feather(self._param_cache_path())
            with open(self._param_meta_path(), "w") as f:
                json.dump(self._param_meta, f)

        # Every call to save() should be preceded by a call to try_load()
        self._key = None
        self._log_path = None
        self._data_meta = None
        self._param_meta = None


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
        profiler.add_function(load_log)
        profiler.add_function(parse_log_data)
        profiler.add_function(parse_log_params)
        profiler.add_function(new_row)
        profiler.enable_by_count()

    if args.fields is None or len(args.fields) == 0:
        raise ValueError(
            "No fields provided. Use --fields to specify message types and/or fields."
        )

    data, params = load_log(
        args.path, args.fields, args.frequency, args.cache_dir
    )
    print(data.head())
    print("...")
    print(data.tail())
    print(params.head())
    print("...")
    print(params.tail())

    # Drop all params that don't change
    # Params is a multi-index with Name, Timestamp. Drop everything with only
    # one timestamp
    params = params.groupby("Name").filter(lambda x: len(x) > 1)

    # Print all the names remaining, and their counts
    print("Remaining params:")
    print(params.groupby("Name").size())
    print("...")

    if args.profile:
        profiler.print_stats()


if __name__ == "__main__":
    main()
