# FAULHABER Gripper Control

This ROS 2 package controls a FAULHABER-driven gripper over CANopen. It provides:

- a ROS 2 node that accepts timestamped `JointTrajectory` chunks and tracks them
  in Cyclic Synchronous Position (CSP) mode;
- a ROS-independent Python API for applications that want to control the motor
  directly;
- repeatable mechanical hard-stop calibration with persistent travel limits;
- measured width, motor current, and waypoint tracking-error telemetry; and
- a read-only connection checker for diagnosing SocketCAN and CANopen without
  enabling or moving the drive.

To assemble the hardware, start with the
[hardware build guide](docs/build-guide/README.md). To get the software
running, jump to the [Quickstart](#quickstart). The
[API and protocol reference](docs/api/README.md) documents the ROS node,
Python interface, low-level CANopen communication, and a
[CAN and CANopen glossary](docs/api/README.md#can-and-canopen-glossary).

The user-facing gripper convention is:

| Quantity | Convention |
|---|---|
| Closed width | `0.0 mm` in the Python API, `0.0 m` in ROS |
| Open width | `max_width_mm`, `81.2 mm` by default |
| Raw closed coordinate | Calibrated `safe_min` |
| Raw open coordinate | Calibrated `safe_max` |

## Architecture

```text
JointTrajectory publisher
          |
          v
ROS callback: validate, convert metres -> millimetres, merge trajectory chunks
          |
          v
Thread-safe waypoint buffer <---- ROS time / simulated time
          |
          v
200 Hz CSP worker -------------> monotonic scheduling and lateness checks
          |
          v
FaulhaberCSPMotor: limits, calibration conversion, lifecycle, safe teardown
          |
          v
python-can / SocketCAN ---- CANopen RPDO + SYNC / TPDO feedback ---- Drive
          ^                                                        |
          +------ statusword, position, and motor current ----------+
```

## Vendor documentation

The exact vendor revisions used to document this implementation are stored in
[`docs/`](docs/README.md):

- [CANopen Communication / Function Manual 7000.05037](docs/EN_7000_05037_CANopen_Communication_Function_Manual.pdf),
  2nd edition, 2018-11-09; and
- [2250 ... BX4 IMC RS/CO product datasheet](docs/EN_2250_BX4_IMC_RSCO_DFF.pdf),
  2026-07-28 edition.

Inline citations below link to the first relevant PDF page. The
[`docs` index](docs/README.md) contains a page map and SHA-256 checksum for each
unchanged source document.

## Quickstart

This path covers a new checkout through first commanded motion. It assumes:

- ROS 2, `colcon`, and `rosdep` are installed and the ROS installation is
  sourced;
- the motor/controller is powered and wired to a Linux CAN adapter according
  to the exact product datasheet;
- the CAN bus is terminated with 120 ohms at each physical end; and
- the default controller settings are node ID `1` and `500000` bit/s.

Replace the interface, node ID, or bitrate in the commands below if your setup
differs. Only one program should control a CANopen node at a time.

### 1. Clone, build, and source

If the repository is not already in a ROS 2 workspace, clone it into the
workspace's `src` directory:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/anunthramaswami2030/franka_gripper_control.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select franka_gripper_control
source install/setup.bash
```

The runtime dependencies are ROS 2 `rclpy`, `trajectory_msgs`,
`sensor_msgs`, `std_msgs`, `std_srvs`, and Python `python-can`. The host also
needs `iproute2` to inspect/configure SocketCAN; `can-utils` is optional but
useful for diagnosis.

### 2. Bring up the CAN interface

SocketCAN is the Linux interface to a CAN adapter; it exposes the bus as a
network device such as `can0`. Configure it before starting the node. Every
device on the physical bus must use the same bitrate:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details -statistics link show can0
```

A healthy idle interface normally reports `can state ERROR-ACTIVE` with zero
live transmit and receive error counters. `ERROR-WARNING`, `ERROR-PASSIVE`, or
`BUS-OFF` usually indicates a bitrate, wiring, termination, power, or node
configuration problem.

### 3. Check the connection without moving the motor

Stop other software that accesses this CANopen node, then run:

```bash
python3 -m franka_gripper_control.check_faulhaber_connection \
  --interface can0 \
  --node-id 1
```

Continue only when the final line reports `RESULT: HEALTHY`. The checker reads
controller state and CAN error counters but does not enable or move the drive.

The checker:

1. records the kernel CAN state and error counters;
2. sends a node-guard remote request
   ([CANopen manual, pp. 30–31](docs/EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=30));
3. reads device type, statusword, active mode, position, velocity, and current
   with expedited SDO uploads
   ([CANopen manual, pp. 22–24](docs/EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=22));
4. passively observes boot-up, emergency, and CAN error frames
   ([CANopen manual, pp. 25–26](docs/EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=25)); and
5. verifies that kernel error counters did not increase.

It does **not** send NMT state changes, Controlwords, mode writes, PDOs, SYNC,
or motion targets. A healthy result verifies communication; it does not verify
that motion is mechanically safe.

Useful checker options are:

| Option | Default | Meaning |
|---|---:|---|
| `--interface` | `can0` | SocketCAN interface |
| `--node-id` | `1` | CANopen node ID, from 1 through 127 |
| `--samples` | `10` | Number of statusword samples |
| `--interval` | `0.5` s | Passive observation time after each sample |
| `--response-timeout` | `1.0` s | Per-request timeout |

### 4. Start the ROS node

Create the default calibration directory if this is the first run:

```bash
mkdir -p ~/.ros
```

Start the node with the normal ROS clock:

```bash
ros2 launch franka_gripper_control faulhaber_gripper.launch.xml
```

The node first tries to load the existing calibration file and enter CSP. If
that fails—for example, on first use when the file does not exist—the ROS node
remains available but ignores trajectories until calibration succeeds.

### 5. Calibrate once

If a valid calibration file already exists for this exact mechanism, skip this
step. Otherwise, clear the complete gripper travel, keep an emergency stop
available, and run from another sourced terminal:

```bash
ros2 service call \
  /faulhaber_gripper/calibrate \
  std_srvs/srv/Trigger \
  "{}"
```

Wait for `success: true`. The default launch file saves calibration at:

```text
~/.ros/faulhaber_gripper_limits.json
```

Do not reuse this file for a different gripper or after changing gripper tips.

### 6. Send a first trajectory

Publish a two-point test trajectory from another sourced terminal. ROS joint
positions are in **metres**, so `0.04` means a 40 mm opening. Replace it with a
known-safe width if 40 mm is not appropriate for your mechanism:

```bash
ros2 topic pub --once \
  /polyumi/target_gripper \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [fr3_gripper_width], points: [
    {positions: [0.0], time_from_start: {sec: 0, nanosec: 0}},
    {positions: [0.04], time_from_start: {sec: 2, nanosec: 0}}
  ]}"
```

For the first accepted trajectory after startup or calibration, the node
replaces the first point's width with the measured current width. Its timestamp
is retained, so the second point above produces a smooth two-second motion from
the physical starting position to `0.04 m`. A one-point first message only
establishes a hold and therefore does not move the gripper.

Confirm measured width and motor current in separate sourced terminals:

```bash
ros2 topic echo /faulhaber_gripper/joint_states
ros2 topic echo /faulhaber_gripper/motor_current_ma
```

### 7. Shut down

Press `Ctrl+C` in the launch terminal. Normal shutdown stops the CSP scheduler,
disables the drive, restores temporary PDO mappings and current limits, and
closes the CAN connection. After the node has exited, the host interface can be
taken down if desired:

```bash
sudo ip link set can0 down
```
