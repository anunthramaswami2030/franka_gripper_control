# FAULHABER gripper API and protocol reference

[Project overview and Quickstart](../../README.md) ·
[Hardware build guide](../build-guide/README.md)

## ROS node API

### Subscribed topic

| Default topic | Type | Units | Description |
|---|---|---|---|
| `/polyumi/target_gripper` | `trajectory_msgs/msg/JointTrajectory` | metres in `positions` | Timestamped chunks containing the configured `fr3_gripper_width` joint |

The execution time of each point is:

```text
message.header.stamp + point.time_from_start
```

If `header.stamp` is zero, receipt time is used as the header time. The callback
selects only the configured joint; velocity, acceleration, and effort fields in
the message are ignored.

An incoming chunk is accepted only when:

- it contains the configured joint and at least one point;
- every selected position is finite;
- point times are strictly increasing;
- every finite width has been clamped into `[0, max_width_mm]`; and
- each segment's estimated motor speed is below
  `max_command_speed_rpm`.

Out-of-range finite widths are clamped with a throttled warning. Structural,
non-finite, timestamp, and speed-validation failures reject the complete
message; the worker never receives a partially accepted trajectory.

New receding-horizon chunks replace buffered points at or after the new
chunk's first timestamp. Older, non-overlapping points are retained. If a new
chunk arrives after the old plan ended but begins in the future, the node
inserts a synthetic hold at receipt time rather than interpolating through a
period in which the new plan was unknown. Before the first point and after the
last point, endpoint widths are held.

### Published topics

| Default topic | Type | Rate | Units | Description |
|---|---|---:|---|---|
| `/faulhaber_gripper/joint_states` | `sensor_msgs/msg/JointState` | CSP rate | metres | Measured opening in `position[0]`; `velocity` and `effort` are empty |
| `/faulhaber_gripper/motor_current_ma` | `std_msgs/msg/Float64` | CSP rate | mA | Signed measured motor current; it is not joint effort |
| `/faulhaber_gripper/waypoint_tracking_error_mm` | `std_msgs/msg/Float64` | At source waypoints | mm | Desired source width minus measured width when a waypoint time is crossed |

The `JointState.name` entry is the configured joint name. Synthetic points and
interpolated CSP samples do not produce waypoint-error messages.

### Calibration service

| Default service | Type | Description |
|---|---|---|
| `/faulhaber_gripper/calibrate` | `std_srvs/srv/Trigger` | Stop tracking, find both stops, save limits, re-enter CSP, and hold position |

The service is synchronous and can take several minutes. While it runs, the
CSP thread is stopped and the old trajectory buffer is cleared. A failed
calibration requests a motor stop and returns `success: false` with the error.

### Time and scheduling

The control worker schedules cycles against `time.monotonic()`. It evaluates
the trajectory at:

```text
ROS now + delay_compensation_ms
```

This split matters during rosbag playback: the motor loop continues at its
real CSP frequency, while the desired trajectory follows `/clock`. A backward
ROS-time jump resets waypoint-error evaluation so errors from the previous time
epoch are not replayed.

For rosbag playback:

```bash
ros2 launch franka_gripper_control faulhaber_gripper.launch.xml \
  use_sim_time:=true
```

Then, in another terminal:

```bash
ros2 bag play /path/to/rosbag_directory --clock 200
```

Future waypoints must be available far enough ahead for delay compensation to
be effective. If `/clock` is paused, the CSP worker continues holding the width
selected at the paused ROS time.

### Launch arguments

Launch overrides use underscore names, for example `node_id:=2`. Except for
`use_sim_time`, these are forwarded as command-line options rather than ROS
parameters.

#### ROS, CAN, and interface names

| Launch argument | Default | Meaning |
|---|---:|---|
| `use_sim_time` | `false` | Use `/clock` instead of the system ROS clock |
| `interface` | `can0` | SocketCAN interface |
| `node_id` | `1` | CANopen node ID, 1–127 |
| `topic` | `/polyumi/target_gripper` | Input trajectory topic |
| `joint_name` | `fr3_gripper_width` | Selected and published joint name |
| `calibration_file` | `~/.ros/faulhaber_gripper_limits.json` | Persistent calibration record |
| `calibration_service` | `/faulhaber_gripper/calibrate` | Trigger service name |
| `joint_state_topic` | `/faulhaber_gripper/joint_states` | Measured width output |
| `current_topic` | `/faulhaber_gripper/motor_current_ma` | Current output |
| `waypoint_error_topic` | `/faulhaber_gripper/waypoint_tracking_error_mm` | Waypoint error output |

#### Calibration and geometry

| Launch argument | Default | Units | Meaning |
|---|---:|---:|---|
| `max_width_mm` | `81.2` | mm | Physical width assigned to safe calibrated travel |
| `first_speed_rpm` | `100` | motor rpm | Coarse hard-stop approach speed |
| `second_speed_rpm` | `10` | motor rpm | Slower repeatability approach speed |
| `current_limit_ma` | `300` | mA | Requested calibration current ceiling |
| `stall_current_ma` | `220` | mA | Loaded-contact threshold; must be below the current ceiling |
| `max_open_travel` | `1000000` | increments | Positive travel bound while searching for the open stop |
| `max_close_travel` | `2000000` | increments | Negative travel bound while searching for the closed stop |
| `homing_timeout` | `180` | s | Timeout for each calibration approach or move |

The requested current limit never raises a lower continuous or peak current
limit already stored in the controller.

#### CSP and validation

| Launch argument | Default | Units | Meaning |
|---|---:|---:|---|
| `csp_current_limit_ma` | `400` | mA | Requested temporary CSP current ceiling |
| `csp_rate_hz` | `200` | Hz | Command, feedback, and telemetry-sample rate |
| `delay_compensation_ms` | `13.3` | ms | Fixed trajectory look-ahead |
| `max_tracking_error_mm` | `0` | mm | Shutdown threshold; zero disables following-error shutdown |
| `max_command_speed_rpm` | `4000` | motor rpm | Preflight estimated segment-speed limit |

With following-error shutdown disabled, an object may prevent the gripper from
reaching the requested width while the controller limits motor current. Enable
a nonzero tracking limit only if contact-induced following error should stop
the system.

List launch arguments with:

```bash
ros2 launch franka_gripper_control faulhaber_gripper.launch.xml --show-args
```

Some advanced parser options are not exposed by the launch file. Use the
executable directly when these must be changed:

```bash
ros2 run franka_gripper_control faulhaber_trajectory_node \
  --interface can0 \
  --node-id 1 \
  --calibration-file "$HOME/.ros/faulhaber_gripper_limits.json" \
  --feedback-timeout-ms 4.0 \
  --max-cycle-lateness-ms 20.0
```

| Direct-only option | Default | Meaning |
|---|---:|---|
| `--backoff-increments` | `100` | Calibration distance moved away between approaches |
| `--stall-duration` | `0.4` s | Required duration of loaded, stopped feedback |
| `--endpoint-margin` | `100` increments | Distance excluded from each mechanical stop |
| `--feedback-timeout-ms` | `4.0` ms | Maximum TPDO wait after each SYNC |
| `--telemetry-hz` | `5.0` Hz | Console telemetry rate |
| `--max-cycle-lateness-ms` | `20.0` ms | Fatal CSP scheduler lateness threshold |
| `--increments-per-revolution` | `3000.0` | Encoder scale used only for speed preflight |
| `--raw-span-estimate` | `121000.0` | Conservative stroke estimate used only for speed preflight |

Use `--help` on the executable for the authoritative option list.

### Inspect ROS interfaces

```bash
ros2 topic echo /faulhaber_gripper/joint_states
ros2 topic echo /faulhaber_gripper/motor_current_ma
ros2 topic echo /faulhaber_gripper/waypoint_tracking_error_mm
ros2 topic hz /faulhaber_gripper/joint_states
ros2 service list | grep faulhaber_gripper
```

## Connection checker

`python3 -m franka_gripper_control.check_faulhaber_connection` checks kernel CAN
state and error counters, requests a node-guard reply, and reads device type,
statusword, mode, position, velocity, and current through SDO uploads. It also
observes boot-up, emergency, and CAN error frames, then checks that kernel error
counters have not increased. See the vendor descriptions of
[node guarding](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=30),
[SDO uploads](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=22),
and [emergency frames](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=25).

It sends no NMT state changes, Controlwords, mode writes, PDOs, SYNC, or motion
targets. `RESULT: HEALTHY` verifies communication, not mechanical safety.

| Option | Default | Meaning |
|---|---:|---|
| `--interface` | `can0` | SocketCAN interface |
| `--node-id` | `1` | CANopen node ID, from 1 through 127 |
| `--samples` | `10` | Number of statusword samples |
| `--interval` | `0.5` s | Passive observation time after each sample |
| `--response-timeout` | `1.0` s | Per-request timeout |

## Python motor API

Applications that do not need ROS can use `FaulhaberCSPMotor` directly. This
class owns the SocketCAN connection, calibration, temporary PDO mappings,
current limits, and restoration.

### Main types

```python
from franka_gripper_control.faulhaber_csp_motor import (
    CSPOptions,
    CalibrationOptions,
    DriveError,
    FaulhaberCSPMotor,
    Feedback,
)
```

| Type or method | Purpose |
|---|---|
| `CalibrationOptions` | Hard-stop speeds, currents, travel bounds, detection thresholds, endpoint margin, and physical width |
| `CSPOptions` | Runtime current ceiling and synchronous feedback timeout |
| `Feedback` | Immutable statusword, raw position, width in mm, and current in mA |
| `FaulhaberCSPMotor.start()` | Open SocketCAN, calibrate or load limits, configure CSP, and return the first held-position feedback |
| `cycle_width_mm(width)` | Preferred operation: send one width target and return its synchronous feedback |
| `command_width_mm(width)` | Send target and SYNC without consuming the resulting feedback |
| `read_feedback()` | Consume one TPDO4 generated by the preceding SYNC |
| `command_raw(target)` | Send a calibrated raw target; still enforces `safe_min`/`safe_max` |
| `calibrate_and_restart()` | Leave CSP, recalibrate, save limits, and restart CSP |
| `stop(quick=False)` | Disable the drive, restore temporary settings, and close the bus |

### Use an existing calibration

`FaulhaberCSPMotor` defaults to `recalibrate=True`, so applications that want
to reuse a file must explicitly pass `recalibrate=False`. This example holds
the measured starting width for two seconds:

```python
import time
from pathlib import Path

from franka_gripper_control.faulhaber_csp_motor import (
    CSPOptions,
    FaulhaberCSPMotor,
)


motor = FaulhaberCSPMotor(
    interface="can0",
    node_id=1,
    calibration_path=Path.home() / ".ros" / "faulhaber_gripper_limits.json",
    recalibrate=False,
    csp=CSPOptions(
        current_limit_ma=400,
        feedback_timeout_ms=4.0,
    ),
)

failed = False
try:
    feedback = motor.start()
    print(f"Started at {feedback.width_mm:.2f} mm")

    period = 1.0 / 200.0
    next_cycle = time.monotonic()
    target_mm = feedback.width_mm

    for cycle in range(400):
        next_cycle += period
        feedback = motor.cycle_width_mm(target_mm)
        if cycle % 40 == 0:
            print(
                f"width={feedback.width_mm:.2f} mm, "
                f"current={feedback.current_ma} mA"
            )
        time.sleep(max(0.0, next_cycle - time.monotonic()))
except BaseException:
    failed = True
    raise
finally:
    motor.stop(quick=failed)
```

The caller owns scheduling and target interpolation: `cycle_width_mm()` sends
each target immediately, with no ramp or ROS segment-speed check. To move,
generate targets from the measured starting width at the desired speed.
The call blocks until TPDO feedback arrives or the configured timeout expires.
A single thread should own one
`FaulhaberCSPMotor`; the class does not serialize concurrent callers. A
production scheduler should also detect accumulated lateness instead of merely
skipping sleep when a cycle overruns, as the ROS worker does.

Prefer `cycle_width_mm()` over separate `command_width_mm()` and
`read_feedback()` calls. Every command sends a SYNC that requests a TPDO4, so
failing to consume feedback can leave old TPDO frames queued on the bus.

### Calibrate from Python

This example starts calibration immediately and physically moves to both
stops:

```python
from pathlib import Path

from franka_gripper_control.faulhaber_csp_motor import (
    CalibrationOptions,
    FaulhaberCSPMotor,
)


motor = FaulhaberCSPMotor(
    interface="can0",
    node_id=1,
    calibration_path=Path.home() / ".ros" / "faulhaber_gripper_limits.json",
    recalibrate=True,
    calibration=CalibrationOptions(
        current_limit_ma=300,
        stall_current_ma=220,
        first_speed_rpm=100,
        second_speed_rpm=10,
        max_open_travel=1_000_000,
        max_close_travel=2_000_000,
        max_width_mm=81.2,
    ),
)

try:
    feedback = motor.start()
    print(f"Calibration complete at {feedback.width_mm:.2f} mm")
finally:
    motor.stop()
```

### Calibration record and coordinate conversion

The JSON record contains the measured stops, safe limits, endpoint margin,
repeatability, maximum physical width, node/interface metadata, and calibration
time. On load, the API requires at least:

```json
{
  "safe_min": -120000,
  "safe_max": -100,
  "max_width_mm": 81.2
}
```

The numbers above are illustrative only; never copy another mechanism's raw
limits. The actual file is generated by calibration.

Conversion is linear across the safe interval:

```text
width_mm = (raw - safe_min) * max_width_mm / (safe_max - safe_min)

raw = round(safe_min + width_mm * (safe_max - safe_min) / max_width_mm)
```

The controller's underlying conversion between internal encoder increments and
the user-unit Position Actual Value is described separately in the Factor Group
section of the vendor manual
([CANopen manual, pp. 46–49](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=46)).
This package's width mapping is an additional application-level calibration
between the two safe gripper endpoints.

The API validates physical widths and raw targets before transmitting them.
This software check complements, but does not replace, controller current
limits and appropriately designed mechanical stops.

## Hard-stop calibration sequence

Calibration uses Profile Position
([CANopen manual, pp. 50–54](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=50)),
Homing
([pp. 55–62](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=55)),
and Profile Velocity
([pp. 63–65](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=63))
modes. The temporary current limits are object `0x2333`
([pp. 77–78](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=77)).
The complete procedure is:

1. Save the controller's continuous current, peak current, acceleration, and
   deceleration.
2. Apply the lower of the requested and already stored current limits; set
   acceleration and deceleration to `100 1/s^2` temporarily.
3. In Profile Velocity mode, approach the opening stop at the first speed.
4. Confirm contact only when current is above the threshold and velocity is
   low for the required duration, while enforcing direction, travel, and time
   bounds.
5. Back away in Profile Position mode and approach the opening stop again at
   the second speed.
6. Use homing method 35 to define that fully open position as raw zero.
7. Approach the closing stop twice, backing away between attempts. The two
   detections must agree within the repeatability tolerance.
8. Average the closing measurements, remove an endpoint margin from both
   mechanical stops, and move away from contact to the safe closed coordinate.
9. Restore the original current, acceleration, and deceleration settings in a
   `finally` path, including after a failed calibration.

Contact detection ignores the first 300 ms of an approach so acceleration is
not mistaken for a stall. It requires all of the following simultaneously:

- absolute current at or above `stall_current_ma`;
- absolute velocity at or below `stall_velocity_rpm`;
- position variation within `stall_position_span`; and
- persistence for `stall_duration`.

## Low-level CANopen protocol

This section describes the frames sent by `FaulhaberCSPMotor` for debugging
and integration. For the CANopen introduction and identifier distribution, see
[manual pp. 15–18](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=15).

All multi-byte values are little-endian. CAN identifiers are standard 11-bit
identifiers; no extended frames are used.

### CAN and CANopen glossary

| Term | Brief definition |
|---|---|
| CAN / CANopen | The physical shared bus / its device-configuration and control protocol. All devices must use the same bitrate, here `500000` bit/s. |
| SocketCAN | Linux CAN interface (`can0`), accessed here through `python-can`. |
| Node ID / COB-ID | Device address (`1`–`127`) / CAN identifier assigned to a message type. This package defaults to node `1`; the table below gives its message IDs. |
| Object dictionary | Indexed controller parameters: `0x6041:00` is index `0x6041`, subindex `0` (Statusword). |
| SDO | Request/reply parameter access for setup and diagnostics. A host read is an *upload*; a write is a *download*. |
| PDO / mapping | Cyclic data frame / the ordered fields and bit lengths packed into it. RPDOs enter the drive; TPDOs leave it. |
| SYNC | Broadcast that applies the staged RPDO command and triggers synchronous TPDO feedback. |
| NMT | Network Management: starts, stops and resets node communication. |
| Heartbeat / node guarding | Periodic node-liveness report / host-requested reply. The checker uses a CAN remote frame for node guarding. |
| EMCY | Asynchronous drive fault report, observed by the checker. |
| CiA 402 | Drive-state and operating-mode profile; Controlword requests transitions, Statusword reports state and faults. |
| CSP | Cyclic Synchronous Position, mode `8`: one position target per cycle. |
| `U8`, `U16`, `U32` / `I16`, `I32` | Unsigned / signed integers with the indicated bit width. |
| `ERROR-ACTIVE` / `BUS-OFF` | Normal CAN-controller state / disconnected after severe repeated errors. `ERROR-WARNING` and `ERROR-PASSIVE` indicate increasing errors. |

### CAN identifiers

For node ID `N`:

| COB-ID | Direction | Purpose in this package |
|---:|---|---|
| `0x000` | Host -> network | NMT command; `01 N` starts node `N` |
| `0x080` | Host -> network | Zero-length CANopen SYNC object |
| `0x080 + N` | Drive -> host | Emergency object, observed by the checker |
| `0x200 + N` | Host -> drive | Temporarily configured synchronous RPDO1: Controlword + Target Position |
| `0x480 + N` | Drive -> host | Temporarily configured synchronous TPDO4: Statusword + Position + Current |
| `0x580 + N` | Drive -> host | SDO response |
| `0x600 + N` | Host -> drive | SDO request |
| `0x700 + N` | Both | Boot-up/heartbeat response and node-guard remote request |

SDO identifiers follow the predefined connection set. The code configures
RPDO1 and TPDO4 COB-IDs temporarily and restores them at shutdown. Vendor
references: [identifiers and PDOs, pp. 17–21](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=17),
[SYNC, p. 27](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=27),
[NMT/boot-up, pp. 28–30](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=28).

### Expedited SDO reads and writes

This implementation uses eight-byte expedited SDOs. Byte 0 is the command
specifier, bytes 1–2 the little-endian object index, byte 3 the subindex, and
bytes 4–7 the value or padding:

| Operation | Command specifier |
|---|---|
| Read request | `0x40` |
| Read response, 1/2/3/4 valid data bytes | `0x4F` / `0x4B` / `0x47` / `0x43` |
| Write request, U8/U16/I32 or U32 | `0x2F` / `0x2B` / `0x23` |
| Successful write response | `0x60` |
| Abort response | `0x80`; bytes 4–7 contain the 32-bit abort code |

Complete layouts and abort codes: [manual pp. 22–24](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=22).

### Object dictionary entries used

| Index | Subindex | Access | Use |
|---:|---:|---|---|
| `0x1000` | `0` | read | Device type in the connection checker |
| `0x1400` | `1`, `2` | read/write | RPDO1 COB-ID and transmission type |
| `0x1600` | `0...` | read/write | RPDO1 mapping count and entries |
| `0x1803` | `1`, `2` | read/write | TPDO4 COB-ID and transmission type |
| `0x1A03` | `0...` | read/write | TPDO4 mapping count and entries |
| `0x2333` | `1`, `2` | read/write | FAULHABER continuous and peak current limits |
| `0x6040` | `0` | read/write | CiA 402 Controlword |
| `0x6041` | `0` | read | CiA 402 Statusword |
| `0x6060` | `0` | read/write | Requested mode of operation |
| `0x6061` | `0` | read | Active mode of operation |
| `0x6064` | `0` | read | Position Actual Value |
| `0x606C` | `0` | read | Velocity Actual Value |
| `0x6078` | `0` | read | Current Actual Value in mA |
| `0x607A` | `0` | read/write | Target Position |
| `0x6083` | `0` | read/write | Profile Acceleration |
| `0x6084` | `0` | read/write | Profile Deceleration |
| `0x6098` | `0` | read/write | Homing Method; method 35 sets current position as zero |
| `0x60FF` | `0` | read/write | Target Velocity in rpm |

Vendor object dictionaries: [communication, pp. 94–100](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=94),
[FAULHABER-specific, pp. 101–106](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=101),
[CiA 402, pp. 107–111](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=107).

Mode values written to `0x6060` are:

| Value | Mode | Use here |
|---:|---|---|
| `1` | Profile Position | Backoff and endpoint moves |
| `3` | Profile Velocity | Guarded hard-stop approaches |
| `6` | Homing | Establish the open position as zero |
| `8` | Cyclic Synchronous Position | Runtime trajectory tracking |

Mode values and active-mode readback (`0x6061`): [manual p. 45](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=45).

### CiA 402 Controlwords and Statusword checks

The drive-enable sequence writes these Controlwords to `0x6040` and checks
Statusword after each transition ([manual pp. 37–44](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=37)):

| Controlword | Meaning | Required masked Statusword |
|---:|---|---:|
| `0x0006` | Shutdown / Ready to Switch On | `0x0021` |
| `0x0007` | Switch On | `0x0023` |
| `0x000F` | Enable Operation | `0x0027` |

The code masks Statusword with `0x006F` before comparing the state. Statusword
bit 3 (`0x0008`) is treated as a fault in every motion mode.

Additional Controlwords are mode-dependent:

| Value | Use |
|---:|---|
| `0x0000` | Disable the drive |
| `0x0002` | Request Quick Stop after a control/startup failure |
| `0x001F` | Start homing while Operation Enabled |
| `0x003F` | Absolute Profile Position: new setpoint + change immediately |
| `0x002F` | Clear the absolute new-setpoint bit |
| `0x007F` | Relative Profile Position: relative + new setpoint + change immediately |
| `0x006F` | Clear the relative new-setpoint bit |

In Profile Position mode, Statusword bit 12 acknowledges a new setpoint and bit
10 indicates target reached. In Homing mode, bit 12 indicates homing attained,
bit 10 indicates the target is reached, and bit 13 indicates a homing error.

### PDO remapping for CSP

Before remapping a PDO, the code follows the CANopen mapping sequence:

1. snapshot the original COB-ID, transmission type, and mappings;
2. set COB-ID bit 31 to disable the PDO;
3. write mapping count zero;
4. write each mapping entry;
5. write the new mapping count;
6. set transmission type `1`, synchronous on every SYNC; and
7. write the active COB-ID with bit 31 clear.

RPDO1 mapping at object `0x1600`:

| Entry | Mapping value | Decoded field |
|---:|---:|---|
| 1 | `0x60400010` | `0x6040:00`, 16-bit Controlword |
| 2 | `0x607A0020` | `0x607A:00`, 32-bit Target Position |

TPDO4 mapping at object `0x1A03`:

| Entry | Mapping value | Decoded field |
|---:|---:|---|
| 1 | `0x60410010` | `0x6041:00`, 16-bit Statusword |
| 2 | `0x60640020` | `0x6064:00`, 32-bit Position Actual Value |
| 3 | `0x60780010` | `0x6078:00`, 16-bit Current Actual Value |

Mappings pack `index`, `subindex`, and bit length; RPDO1 is six bytes and TPDO4
eight. Transmission types and remapping: [manual pp. 18–21](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=18).

### One CSP command/feedback cycle

After the drive is in CSP and Operation Enabled, one cycle is:

1. Convert width in millimetres to a signed 32-bit raw position.
2. Verify the raw position lies inside `[safe_min, safe_max]`.
3. Send synchronous RPDO1 to `0x200 + node_id`:

   ```text
   byte 0..1: U16 Controlword = 0x000F
   byte 2..5: I32 Target Position
   ```

4. Send a zero-length SYNC frame on ID `0x080`. The drive applies the staged
   RPDO and emits TPDO4 because both PDO transmission types are `1`.
5. Receive TPDO4 from `0x480 + node_id`:

   ```text
   byte 0..1: U16 Statusword
   byte 2..5: I32 Position Actual Value
   byte 6..7: I16 Current Actual Value in mA
   ```

6. Reject feedback if fault bit 3 is set or the masked state is not Operation
   Enabled (`0x0027`).

SYNC and synchronous PDO behavior: [manual p. 27](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=27).

Illustrative node-1 frames for raw target `-12345` and feedback current
`+250 mA` are:

```text
RPDO1: ID 0x201  DATA 0F 00 C7 CF FF FF
SYNC:  ID 0x080  DATA <empty>
TPDO4: ID 0x481  DATA 27 00 C7 CF FF FF FA 00
```

These bytes are an encoding example, not safe values for a particular gripper.
Do not send them with `cansend`: CSP mode, PDO mappings, current limits, and
calibrated travel must already be correct.

### Startup sequence

`FaulhaberCSPMotor.start()` performs the following:

1. Open the SocketCAN bus and read current Statusword, mode, and position.
2. Send NMT Start Remote Node: CAN ID `0x000`, data `01 node_id`.
3. Select Profile Position mode and transition through Controlwords `0x0006`,
   `0x0007`, and `0x000F`.
4. Calibrate or load the calibration record.
5. Save continuous and peak current limits and apply the lowest of the stored
   limits and requested CSP limit.
6. Disable the drive, read current position, and write that position as the
   first target so CSP entry cannot cause a setpoint jump.
7. Select mode 8, configure RPDO1 and TPDO4, and repeat the CiA 402 enable
   transition.
8. Send the current position followed by SYNC and read the first TPDO4.

Vendor references: [NMT Start, pp. 28–29](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=28),
[drive-enable example, p. 41](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=41).

If any step fails, `start()` requests a quick stop where possible and executes
the normal restoration path.

### Shutdown and restoration

`stop()` is idempotent and best-effort. It:

1. optionally writes Quick Stop Controlword `0x0002`;
2. disables the drive with `0x0000`;
3. restores the original TPDO4 mapping;
4. restores the original RPDO1 mapping;
5. restores the original operating mode;
6. restores continuous and peak current limits; and
7. closes the SocketCAN bus.

Quick Stop and Disable Voltage: [manual pp. 40–42](../EN_7000_05037_CANopen_Communication_Function_Manual.pdf#page=40).

A restoration failure is printed but does not prevent the remaining cleanup
steps. Normal ROS shutdown disables the drive without Quick Stop; worker or
startup errors request Quick Stop first.

## Safety behavior and limitations

- Current limits, travel bounds, endpoint margins, timeouts, state checks, and
  raw-target validation are independent safeguards; none replaces appropriate
  hardware safety.
- The node holds its current calibrated width until a valid trajectory arrives
  and holds the final trajectory width after the buffer ends.
- The ROS node clamps out-of-range finite widths into its configured interval
  and emits a throttled warning. Direct Python motor commands reject widths
  outside the calibrated interval.
- Incoming trajectory velocity and acceleration arrays are ignored; motion is
  defined only by width positions and timestamps.
- `max_command_speed_rpm` is a preflight estimate based on a configured stroke
  estimate. It does not configure the controller's own velocity limit.
- Zero `max_tracking_error_mm` permits persistent position error during object
  contact. The temporary CSP current ceiling remains active.
- A TPDO timeout, controller reboot, drive fault, lost Operation Enabled state,
  or excessive scheduler lateness stops the worker and triggers shutdown.
- Calibration is tied to a particular mechanism, transmission, node, and width
  definition. Recalibrate after mechanical changes and do not share limit files
  between grippers.
- Width conversion is linear between calibrated endpoints. Nonlinear linkages
  require a different conversion model.

## Source layout

| File | Responsibility |
|---|---|
| [`faulhaber_joint_trajectory_node.py`](../../franka_gripper_control/faulhaber_joint_trajectory_node.py) | ROS interfaces, trajectory validation/buffering, CSP worker, CLI |
| [`faulhaber_csp_motor.py`](../../franka_gripper_control/faulhaber_csp_motor.py) | CANopen/CiA 402 primitives, calibration, CSP API, restoration |
| [`check_faulhaber_connection.py`](../../franka_gripper_control/check_faulhaber_connection.py) | Read-only SocketCAN and controller health check |
| [`faulhaber_gripper.launch.xml`](../../launch/faulhaber_gripper.launch.xml) | Supported launch arguments and defaults |
| [`docs/`](../README.md) | Bundled FAULHABER manuals, page map, revisions, and checksums |

For the most precise API details, see the NumPy-style docstrings in these
modules and run:

```bash
ros2 run franka_gripper_control faulhaber_trajectory_node --help
```
