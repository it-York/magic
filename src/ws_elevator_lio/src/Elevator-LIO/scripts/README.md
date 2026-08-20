# Scripts

This directory contains utility scripts released together with Elevator-LIO.

## bag_runner_ui.py

`bag_runner_ui.py` is a rosbag manager released together with Elevator-LIO. It is designed to help organize rosbag sequences, run batch tests, manage event flags, and review generated results during algorithm development and evaluation.

This optional evaluation GUI currently targets ROS 1 `.bag` datasets and the ROS 1 command-line tools. It is
not required to build or run the ROS 1/ROS 2 `lio` package. For native ROS 2 recordings, use `ros2 bag` (or
convert the released ROS 1 bags to rosbag2 format) and launch Elevator-LIO with `start_ros2.launch.py`.

Usage video:

[Bilibili: Elevator-LIO rosbag manager usage](https://www.bilibili.com/video/BV1n3jt64Eoi/?share_source=copy_web&vd_source=392db04838f1edf7d12e58a3d68775d8)

Run:

```bash
python3 scripts/bag_runner_ui.py
```

## download_dataset.sh

`download_dataset.sh` downloads the Elevator-LIO dataset from Hugging Face. By default, it downloads the main dataset to a sibling directory:

```text
../Elevator-LIO-Dataset
```

Basic usage:

```bash
./scripts/download_dataset.sh
```

Specify an output directory:

```bash
./scripts/download_dataset.sh --output-dir /data/Elevator-LIO-Dataset
```

Also download community-contributed bags:

```bash
./scripts/download_dataset.sh --include-community
```

Show the SJTU cloud mirror link:

```bash
./scripts/download_dataset.sh --sjtu
```

The script uses `huggingface_hub` and resumes interrupted downloads when rerun with the same options.

## convert_rosbag1_to_rosbag2.py

`convert_rosbag1_to_rosbag2.py` converts the released ROS 1 bags to ROS 2 Humble-compatible SQLite3
rosbag2 directories. It preserves the real Mid-360 driver's ROS 2 type name,
`livox_ros_driver2/msg/CustomMsg`, while converting the ROS 1 bytes to ROS 2 CDR. For the released
Elevator-LIO dataset, all supported topics, timestamps, and message counts are preserved.

Install the ROS-independent conversion library once:

```bash
python3 -m pip install --user rosbags
```

`rosbags` is the converter's only direct Python dependency; pip installs its transitive dependencies
automatically. The script does not import ROS 1 `rosbag`, ROS 2 `rclpy`, or `rosbag2_py`, and no ROS
environment needs to be sourced during conversion. It has been exercised in an isolated Python 3.10
environment with current `rosbags` 0.11.x, matching Ubuntu 22.04 / ROS 2 Humble's default Python version.

Convert one or several sequences (the default input is `../Elevator-LIO-Dataset`):

```bash
./scripts/convert_rosbag1_to_rosbag2.py Office1
./scripts/convert_rosbag1_to_rosbag2.py Campus1 Dormitory1
```

Convert the complete dataset, optionally to a disk with more free space:

```bash
./scripts/convert_rosbag1_to_rosbag2.py --all --output-dir /data/Elevator-LIO-Dataset-rosbag2
```

The script verifies topic counts and CDR decoding before publishing each completed output directory. It also
performs a conservative disk-space check; use `--dry-run` to inspect a conversion without writing anything.
It searches common `~/下载/...` and `~/Downloads/...` driver locations and verifies any detected definitions
against the Livox ROS Driver 2 schema. If neither a driver checkout nor the source repository's definitions
are available, it uses the same official message fields embedded in the script. Another driver checkout can
be selected with `--livox-msg-dir` or the `LIVOX_ROS_DRIVER2_MSG_DIR` environment variable.

The same script is also published in the Hugging Face dataset root. In that location, the dataset directory
itself is the default input, so it can be used on an Ubuntu 22.04 machine that only has ROS 2 Humble installed.

Play a converted sequence after sourcing the ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
source <livox_ros_driver2_ws>/install/setup.bash
source <elevator_lio_ros2_ws>/install/setup.bash
ros2 bag info ../Elevator-LIO-Dataset-rosbag2/Office1
ros2 bag play ../Elevator-LIO-Dataset-rosbag2/Office1
```
