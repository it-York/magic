#!/usr/bin/env python3
"""Convert Elevator-LIO ROS 1 bags to ROS 2 Humble rosbag2 bags.

The released dataset records Livox packets as
``livox_ros_driver2/CustomMsg``.  The converted bag preserves the ROS 2 type
name and schema ``livox_ros_driver2/msg/CustomMsg`` used by the real Mid-360
driver while transcoding the ROS 1 wire format to ROS 2 CDR.  The script works
both from Elevator-LIO's ``scripts/`` directory and from the dataset root.
"""

from __future__ import print_function

import argparse
import inspect
import os
import shutil
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

try:
    from rosbags.highlevel import AnyReader
    from rosbags.rosbag2 import Reader as Rosbag2Reader
    from rosbags.rosbag2 import Writer as Rosbag2Writer
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError as exc:
    print(
        "error: Python package 'rosbags' is required. Install it with:\n"
        "  python3 -m pip install --user rosbags\n"
        "The converter itself does not require a sourced ROS environment.",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
RUNNING_FROM_SOURCE_REPOSITORY = SCRIPT_DIR.name == "scripts"
PACKAGE_ROOT = SCRIPT_DIR.parent if RUNNING_FROM_SOURCE_REPOSITORY else None
DEFAULT_INPUT_DIR = (
    PACKAGE_ROOT.parent / "Elevator-LIO-Dataset"
    if PACKAGE_ROOT is not None
    else SCRIPT_DIR
)
DEFAULT_DRIVER_MSG_DIR = (
    Path.home() / "下载" / "ws_livox_ros_driver2" / "src" / "livox_ros_driver2" / "msg"
)

LIVOX_TYPE = "livox_ros_driver2/msg/CustomMsg"
CUSTOM_MESSAGE_NAMES = ("CustomPoint", "CustomMsg")

# These are the wire-relevant fields from Livox ROS Driver 2.  Keeping an
# embedded copy makes the dataset-hosted script self-contained; when a driver
# checkout is found, its definitions are normalized and checked against these
# fields before use.
EMBEDDED_LIVOX_MESSAGES = {
    "CustomPoint": """\
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
""",
    "CustomMsg": """\
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
""",
}

# CDR plus SQLite indexes is normally close to the uncompressed ROS 1 size.
# Keep a conservative reserve so a conversion does not fill the filesystem.
SPACE_FACTOR = 1.25
SPACE_RESERVE_BYTES = 128 * 1024 * 1024


class ConversionError(RuntimeError):
    """A user-facing conversion or validation error."""


def human_bytes(value: int) -> str:
    units = ("B", "KiB", "MiB", "GiB", "TiB")
    size = float(value)
    for unit in units:
        if size < 1024.0 or unit == units[-1]:
            return "{:.1f} {}".format(size, unit)
        size /= 1024.0
    return "{} B".format(value)


def output_type(source_type: str) -> str:
    # AnyReader normalizes ROS 1 names (pkg/Msg) to ROS 2 spelling
    # (pkg/msg/Msg).  The driver package name must remain unchanged.
    return source_type


def normalize_msg_definition(definition: str) -> str:
    """Return only message fields, ignoring comments and spacing."""
    fields = []
    for line in definition.splitlines():
        field = line.split("#", 1)[0].strip()
        if field:
            fields.append(" ".join(field.split()))
    return "\n".join(fields)


def validate_driver_msg_dir(msg_dir: Path) -> Path:
    resolved = msg_dir.expanduser().resolve()
    for name in CUSTOM_MESSAGE_NAMES:
        msg_path = resolved / "{}.msg".format(name)
        if not msg_path.is_file():
            raise ConversionError("message definition not found: {}".format(msg_path))
        actual = normalize_msg_definition(msg_path.read_text(encoding="utf-8"))
        expected = normalize_msg_definition(EMBEDDED_LIVOX_MESSAGES[name])
        if actual != expected:
            raise ConversionError(
                "{} does not match the Livox ROS Driver 2 {} schema".format(msg_path, name)
            )
    return resolved


def find_driver_msg_dir(explicit_path: Path = None) -> Optional[Path]:
    if explicit_path is not None:
        return validate_driver_msg_dir(explicit_path)

    candidates = []
    environment_path = os.environ.get("LIVOX_ROS_DRIVER2_MSG_DIR", "").strip()
    if environment_path:
        return validate_driver_msg_dir(Path(environment_path))
    candidates.extend(
        (
            DEFAULT_DRIVER_MSG_DIR,
            Path.home()
            / "Downloads"
            / "ws_livox_ros_driver2"
            / "src"
            / "livox_ros_driver2"
            / "msg",
            SCRIPT_DIR / "msg",
        )
    )
    if PACKAGE_ROOT is not None:
        candidates.append(PACKAGE_ROOT / "msg")

    for candidate in candidates:
        if all((candidate / "{}.msg".format(name)).is_file() for name in CUSTOM_MESSAGE_NAMES):
            return validate_driver_msg_dir(candidate)
    return None


def create_ros2_typestore(msg_dir: Optional[Path]):
    store = get_typestore(Stores.ROS2_HUMBLE)
    definitions = {}
    for name in CUSTOM_MESSAGE_NAMES:
        definition = (
            (msg_dir / "{}.msg".format(name)).read_text(encoding="utf-8")
            if msg_dir is not None
            else EMBEDDED_LIVOX_MESSAGES[name]
        )
        definitions.update(
            get_types_from_msg(
                definition,
                "livox_ros_driver2/msg/{}".format(name),
            )
        )
    store.register(definitions)
    return store


def create_writer(path: Path, rosbag2_version: int):
    """Support both rosbags 0.9.x and the current explicit-version API."""
    parameters = inspect.signature(Rosbag2Writer).parameters
    if "version" in parameters:
        return Rosbag2Writer(path, version=rosbag2_version)
    return Rosbag2Writer(path)


def resolve_bag(token: str, input_dir: Path) -> Path:
    candidate = Path(token).expanduser()
    if candidate.is_file():
        return candidate.resolve()

    relative = input_dir / candidate
    if relative.is_file():
        return relative.resolve()

    if candidate.suffix != ".bag":
        relative = input_dir / "{}.bag".format(candidate)
        if relative.is_file():
            return relative.resolve()

    raise ConversionError("ROS 1 bag not found: {}".format(token))


def select_inputs(args) -> List[Path]:
    input_dir = args.input_dir.expanduser().resolve()
    if args.all:
        if args.bags:
            raise ConversionError("do not combine bag names with --all")
        if not input_dir.is_dir():
            raise ConversionError("dataset directory not found: {}".format(input_dir))
        bags = sorted(input_dir.glob("*.bag"))
        if not bags:
            raise ConversionError("no .bag files found in {}".format(input_dir))
        return [path.resolve() for path in bags]

    if not args.bags:
        raise ConversionError(
            "specify one or more bag names (for example: Office1), or use --all"
        )

    selected = []
    seen = set()
    for token in args.bags:
        path = resolve_bag(token, input_dir)
        if path not in seen:
            selected.append(path)
            seen.add(path)
    return selected


def nearest_existing_path(path: Path) -> Path:
    candidate = path
    while not candidate.exists():
        if candidate.parent == candidate:
            raise ConversionError("cannot find an existing parent for {}".format(path))
        candidate = candidate.parent
    return candidate


def check_disk_space(inputs: Sequence[Path], output_root: Path) -> None:
    input_bytes = sum(path.stat().st_size for path in inputs)
    required = int(input_bytes * SPACE_FACTOR) + SPACE_RESERVE_BYTES
    existing_parent = nearest_existing_path(output_root)
    free = shutil.disk_usage(str(existing_parent)).free
    if free < required:
        raise ConversionError(
            "insufficient free space on {}: need approximately {}, only {} available. "
            "Choose another filesystem with --output-dir, convert fewer bags, or use "
            "--ignore-space-check if you have verified the capacity yourself.".format(
                existing_parent, human_bytes(required), human_bytes(free)
            )
        )
    print(
        "Disk check: approximately {} required; {} available on {}".format(
            human_bytes(required), human_bytes(free), existing_parent
        )
    )


def inspect_source(source: Path, ros2_store):
    with AnyReader([source]) as reader:
        if not reader.connections:
            raise ConversionError("bag contains no topics: {}".format(source))

        topic_types = {}
        expected_counts = defaultdict(int)
        connection_specs = []
        for connection in reader.connections:
            target_type = output_type(connection.msgtype)
            previous_type = topic_types.setdefault(connection.topic, target_type)
            if previous_type != target_type:
                raise ConversionError(
                    "topic {} has multiple message types: {} and {}".format(
                        connection.topic, previous_type, target_type
                    )
                )
            if target_type not in ros2_store.types:
                raise ConversionError(
                    "ROS 2 Humble type is unavailable for {}: {}".format(
                        connection.topic, target_type
                    )
                )
            key = (connection.topic, target_type)
            expected_counts[key] += connection.msgcount
            connection_specs.append((connection.id, key))

        return reader.message_count, dict(expected_counts), connection_specs


def print_plan(source: Path, destination: Path, message_count: int,
               expected_counts: Mapping[Tuple[str, str], int]) -> None:
    print("\n{} -> {}".format(source, destination))
    print("  source size: {}; messages: {:,}".format(human_bytes(source.stat().st_size), message_count))
    for (topic, msgtype), count in sorted(expected_counts.items()):
        suffix = " (Livox ROS2 driver type)" if msgtype == LIVOX_TYPE else ""
        print("  {:>8,}  {}  {}{}".format(count, topic, msgtype, suffix))


def remove_generated_path(path: Path) -> None:
    if path.is_dir():
        shutil.rmtree(str(path))
    elif path.exists():
        path.unlink()


def convert_one(source: Path, destination: Path, partial: Path, ros2_store,
                rosbag2_version: int, force: bool) -> Tuple[int, float]:
    if destination.exists():
        if not force:
            print("Skipping existing output: {} (use --force to replace it)".format(destination))
            return 0, 0.0
        remove_generated_path(destination)

    if partial.exists():
        if not force:
            raise ConversionError(
                "incomplete output exists: {}. Inspect it or rerun with --force.".format(partial)
            )
        remove_generated_path(partial)

    message_count, expected_counts, connection_specs = inspect_source(source, ros2_store)
    print_plan(source, destination, message_count, expected_counts)

    partial.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    last_report = started
    processed = 0

    try:
        with AnyReader([source]) as reader:
            with create_writer(partial, rosbag2_version) as writer:
                output_connections = {}
                for key in expected_counts:
                    topic, msgtype = key
                    output_connections[key] = writer.add_connection(
                        topic, msgtype, typestore=ros2_store
                    )

                connection_map = {
                    source_id: output_connections[key] for source_id, key in connection_specs
                }
                type_map = {
                    source_id: key[1] for source_id, key in connection_specs
                }

                for connection, timestamp, rawdata in reader.messages():
                    message = reader.deserialize(rawdata, connection.msgtype)
                    target_type = type_map[connection.id]
                    cdr = ros2_store.serialize_cdr(message, target_type)
                    writer.write(connection_map[connection.id], timestamp, cdr)
                    processed += 1

                    now = time.monotonic()
                    if now - last_report >= 2.0:
                        elapsed = max(now - started, 0.001)
                        percent = 100.0 * processed / max(message_count, 1)
                        print(
                            "  progress: {:6.2f}% ({:,}/{:,}), {:.0f} msg/s".format(
                                percent, processed, message_count, processed / elapsed
                            ),
                            flush=True,
                        )
                        last_report = now
    except BaseException:
        print("Conversion stopped; incomplete output was kept at {}".format(partial), file=sys.stderr)
        raise

    elapsed = time.monotonic() - started
    verify_output(partial, expected_counts, ros2_store)
    os.replace(str(partial), str(destination))
    try:
        partial.parent.rmdir()
    except OSError:
        pass

    print(
        "Completed: {} ({:,} messages in {:.1f}s)".format(destination, processed, elapsed)
    )
    return processed, elapsed


def verify_output(path: Path, expected_counts: Mapping[Tuple[str, str], int], ros2_store) -> None:
    actual_counts = defaultdict(int)
    decoded = set()
    with Rosbag2Reader(path) as reader:
        for connection in reader.connections:
            actual_counts[(connection.topic, connection.msgtype)] += connection.msgcount

        if dict(actual_counts) != dict(expected_counts):
            raise ConversionError(
                "output topic/message counts differ from the source: expected {}, got {}".format(
                    dict(expected_counts), dict(actual_counts)
                )
            )

        for connection, _timestamp, rawdata in reader.messages():
            key = (connection.topic, connection.msgtype)
            if key not in decoded:
                ros2_store.deserialize_cdr(rawdata, connection.msgtype)
                decoded.add(key)
            if len(decoded) == len(expected_counts):
                break

    if decoded != set(expected_counts):
        raise ConversionError("not every output topic contained a decodable message")
    print("  verification: counts and first CDR message of every topic are valid")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Convert Elevator-LIO ROS 1 .bag files to ROS 2 Humble SQLite3 bags, "
            "preserving the Livox driver's CustomMsg type."
        )
    )
    parser.add_argument(
        "bags",
        nargs="*",
        help="bag path or sequence name relative to --input-dir (for example: Office1)",
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=DEFAULT_INPUT_DIR,
        help="dataset directory (default: %(default)s)",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        help="output root (default: <input-dir>-rosbag2)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="convert every top-level .bag in --input-dir",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="replace existing outputs and incomplete outputs",
    )
    parser.add_argument(
        "--ignore-space-check",
        action="store_true",
        help="continue even when the conservative free-space check fails",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="inspect inputs and print the conversion plan without writing",
    )
    parser.add_argument(
        "--livox-msg-dir",
        type=Path,
        help=(
            "directory containing the ROS 2 driver's CustomMsg.msg and CustomPoint.msg "
            "(auto-detected; exact driver schemas are embedded as a fallback)"
        ),
    )
    parser.add_argument(
        "--rosbag2-version",
        type=int,
        choices=(8, 9),
        default=8,
        help="rosbag2 metadata format for recent rosbags releases (8 is the Humble-safe default)",
    )
    return parser


def main(argv: Sequence[str] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        inputs = select_inputs(args)
        input_dir = args.input_dir.expanduser().resolve()
        output_root = (
            args.output_dir.expanduser().resolve()
            if args.output_dir
            else input_dir.with_name(input_dir.name + "-rosbag2")
        )
        destinations = [(source, output_root / source.stem) for source in inputs]
        pending = [source for source, destination in destinations if args.force or not destination.exists()]

        driver_msg_dir = find_driver_msg_dir(args.livox_msg_dir)
        if driver_msg_dir is None:
            print("Livox ROS 2 message definitions: embedded official driver schema")
        else:
            print("Livox ROS 2 message definitions: {} (schema verified)".format(driver_msg_dir))
        ros2_store = create_ros2_typestore(driver_msg_dir)
        plans = []
        for source, destination in destinations:
            message_count, expected_counts, _specs = inspect_source(source, ros2_store)
            plans.append((source, destination, message_count, expected_counts))

        if args.dry_run:
            for source, destination, message_count, expected_counts in plans:
                print_plan(source, destination, message_count, expected_counts)
            if pending and not args.ignore_space_check:
                check_disk_space(pending, output_root)
            print("\nDry run only; no files were written.")
            return 0

        if pending and not args.ignore_space_check:
            check_disk_space(pending, output_root)

        total_messages = 0
        total_elapsed = 0.0
        partial_root = output_root / ".partial"
        for source, destination in destinations:
            processed, elapsed = convert_one(
                source,
                destination,
                partial_root / destination.name,
                ros2_store,
                args.rosbag2_version,
                args.force,
            )
            total_messages += processed
            total_elapsed += elapsed

        print(
            "\nDone: {:,} messages converted in {:.1f}s. Output root: {}".format(
                total_messages, total_elapsed, output_root
            )
        )
        return 0
    except (ConversionError, OSError, ValueError) as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
