# HumanaOpen [![Product Introduction](https://img.shields.io/badge/Product%20Introduction-Visit-blue?style=for-the-badge)](https://caizonglin.github.io/)

<img src="docs/HumanaOpen.png" alt="HumanaOpen" width="100%">

[English](README.md) | [中文](README_zh.md) | [Français](README_fr.md) | [한국어](README_ko.md)

**Open-source semi-humanoid robot — 7-DOF dual arms, differential drive, and leadscrew lift.**

Built on [LeRobot](https://github.com/huggingface/lerobot) and
[open-arms-mini](https://github.com/TheRobotStudio/open-arms-mini).

## Hardware

| Subsystem | Motors | Model |
|-----------|--------|-------|
| Left follower arm | 8 (7-DOF + gripper) | ST3215 C018 (1:345) |
| Right follower arm | 8 (7-DOF + gripper) | ST3215 C018 (1:345) |
| Head (pan/tilt) | 2 | ST3215 C018 (1:345) |
| Lift (leadscrew) | 1 | ST3250 (direct drive, no belt) |
| Differential drive base | 2 | ST3215 C018 (1:345) |
| Leader arms (teleop) | 2 × 8 | STS3215 C046 (1:147) |

> **Left/right convention**: Defined from the robot's own frame of reference.
> Standing behind the robot and facing the same direction as it, the arm on your
> left-hand side is the **left arm** (`port1`), the arm on your right-hand side is
> the **right arm** (`port2`). Wiring decides which physical arm is which; the
> software simply maps `left_arm_*` → `port1` and `right_arm_*` → `port2`.

## Software

```
lerobot_robot_humanaopen/
├── __init__.py              # Package exports
├── config_humanaopen.py     # HumanaOpenConfig, host/client configs
├── humanaopen.py            # HumanaOpen Robot class (follower)
├── lift_axis.py             # Lift axis with stall-detection homing
├── leader.py                # Leader teleoperator (single/bimanual)
├── humanaopen_host.py       # ZMQ host (robot-side, for dual-machine mode)
└── humanaopen_client.py     # ZMQ client (teleop-side)
examples/
├── record_data.py              # Data collection (single + dual-machine)
├── eval_data.py                # Inference (single + dual-machine)
├── teleop_leader_to_follower.py  # Full-body teleop: leader arms + keyboard
├── single_machine.py           # Single machine operation
├── teleop_keyboard.py          # Keyboard teleoperation via ZMQ
├── calibrate_follower.py       # Full follower-side calibration (arms+head+wheels+lift)
├── calibrate_leader.py         # Leader arm calibration (open-arms-mini)
├── diagnose_teleop.py          # Teleop joint direction diagnosis
├── test_base_keyboard.py       # Base-only keyboard test (no lift/arms)
├── test_lift_only.py           # Lift axis test (homing + raise/lower)
└── check_phase.py              # Check servo velocity unit (Phase BIT2)
```

## Hardware

```
hardware/
├── README.md               # Hardware overview (directory guide)
├── BOM/bom.md              # Bill of Materials — parts, specs, sourcing
├── assembly/assembly.md    # Step-by-step build instructions
├── cad/
│   ├── fusion360/          # Fusion 360 source designs (.f3d)
│   └── step/               # STEP exports (.step/.stp)
├── stl/                    # Ready-to-print 3D printing files (.stl)
├── urdf/                   # Robot description for RViz / Gazebo / simulation
│   └── humanaopen.urdf     # (meshes/ holds the referenced STL meshes)
└── electronics/wiring.md   # Wiring / electronics layout and power notes
```

### Diagnostics & Tuning Tools

| Script | Purpose |
|--------|---------|
| `diag_head_tilt_limits.py` | Head tilt mechanical range probe (before/after unlock) |
| `diag_head_tilt_range.py` | Head tilt range diagnostic |
| `diag_regression.py` | Regression test (lift + camera + teleop sequence) |
| `diag_st3250_speed.py` | ST3250 motor speed profiling (Phase BIT2=1) |
| `diag_follower_gripper.py` | Gripper joint diagnosis |
| `recover_lift_ping.py` | Lift motor communication ping |
| `speed_test_bit2_0.py` | BIT2=0 speed verification |
| `switch_phase_bit2.py` | Toggle ST3250 Phase BIT2 register |
```

## Quick Start

```bash
# 1. Create a conda env
conda create -n humanaopen python=3.12
conda activate humanaopen

# 2. Install LeRobot with all required extras (feetech + dataset + training + viz + hardware + smolvla)
pip install "lerobot[dataset,training,feetech,viz,transformers-dep,hardware,smolvla]"

# 3. Install HumanaOpen (editable)
cd /path/to/HumanaOpen
pip install -e . --no-deps

# Required: SOCKS proxy + SmolVLA tokenizer
pip install httpx[socks] num2words

# Optional: GPU with CUDA 12.8+ (Blackwell / RTX 5060+)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128

# 4. Verify installation
python -c "from lerobot_robot_humanaopen import HumanaOpen, HumanaOpenConfig; print('✅ OK')"

# 5. Verify installation (no calibration — just check import + connection)
python -c "
from lerobot_robot_humanaopen import HumanaOpen, HumanaOpenConfig
config = HumanaOpenConfig(port1='/dev/ttyACM0', port2='/dev/ttyACM1', port3=None, cameras={})
robot = HumanaOpen(config)
robot.connect(calibrate=False)
print(robot.get_observation().keys())
"

# 6. Dual-machine ZMQ mode (⚠️ for Jetson/Raspberry Pi deployment only — skip if single machine)
# See "Dual-Machine Deployment" section below for full setup guide.
```

## Dual-Machine Deployment

For production deployment, the robot hardware (servos + cameras) connects to an
embedded board (Jetson or Raspberry Pi), while the policy inference runs on a
separate GPU machine. The two communicate over ZMQ.

```
┌──────────────────────┐       ZMQ TCP        ┌──────────────────────┐
│   Dev Machine (GPU)  │ ←──────────────────→ │  Jetson / RPi (Host) │
│                      │   obs ──────────→    │                      │
│  • Policy inference  │   ←── action (21DOF) │  • Read servos/cams  │
│  • ACT / SmolVLA     │                       │  • Execute actions   │
│  • No servo wiring   │                       │  • Servo + cam wiring│
└──────────────────────┘                       └──────────────────────┘
```

### Raspberry Pi (Host only — no GPU inference)

```bash
# On Raspberry Pi (ARM64)
conda create -n humanaopen python=3.12
conda activate humanaopen

# Host dependencies (lightweight — no torch/transformers needed)
pip install pyzmq feetech-servo-sdk
# Optional: ffmpeg for video encoding (not needed if only streaming via ZMQ)
# conda install -y ffmpeg=7.1.1 -c conda-forge

# Install HumanaOpen
cd ~/ && git clone https://github.com/OneRobotAI/HumanaOpen.git
cd HumanaOpen && pip3 install -e . --no-deps

# Start Host — recommended: launcher script (bus topology + cameras via flags; same as teleop/record/eval scripts)
#   2-bus, no cameras                  python3 examples/humanaopen_host_launcher.py --no-cameras
#   2-bus, with cameras (default)      python3 examples/humanaopen_host_launcher.py
#   3-bus, with cameras                python3 examples/humanaopen_host_launcher.py --robot.port3 /dev/ttyACM2
#   3-bus, no cameras                  python3 examples/humanaopen_host_launcher.py --robot.port3 /dev/ttyACM2 --no-cameras
#   Override a camera device           python3 examples/humanaopen_host_launcher.py --head-camera /dev/video1
#   (--robot.port3 None = 2-bus; any device name = 3-bus. Defaults: head=/dev/video0, left_wrist=/dev/video2, right_wrist=/dev/video4)
#
# Equivalent inline form (also works):
python3 -c "
from lerobot_robot_humanaopen.humanaopen_host import HumanaOpenHost
from lerobot_robot_humanaopen import HumanaOpenConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

HumanaOpenHost(HumanaOpenConfig(
    port1='/dev/ttyACM0', port2='/dev/ttyACM1', port3=None,
    cameras={
        'head':         OpenCVCameraConfig(index_or_path='/dev/video0', fps=30, width=640, height=480, fourcc='MJPG'),  # adjust /dev/videoN to match your board (lerobot-find-cameras)
        'left_wrist':   OpenCVCameraConfig(index_or_path='/dev/video2', fps=30, width=640, height=480, fourcc='MJPG'),  # adjust /dev/videoN to match your board (lerobot-find-cameras)
        'right_wrist':  OpenCVCameraConfig(index_or_path='/dev/video4', fps=30, width=640, height=480, fourcc='MJPG'),
                # 'chest': OpenCVCameraConfig(index_or_path='/dev/video6', fps=30, width=640, height=480, fourcc='MJPG'),  # 4th camera — uncomment to enable
    },
    wheel_dir_signs={'base_left_wheel': -1, 'base_right_wheel': 1}
)).run()
"
```

### NVIDIA Jetson (Host + optional local inference)

```bash
# On Jetson (JetPack 6.x, CUDA 12.x)
# Same installation as Raspberry Pi — Host does not need torch/transformers
conda create -n humanaopen python=3.12
conda activate humanaopen

pip install pyzmq feetech-servo-sdk
# conda install -y ffmpeg=7.1.1 -c conda-forge  # optional

cd ~/ && git clone https://github.com/OneRobotAI/HumanaOpen.git
cd HumanaOpen && pip3 install -e . --no-deps

# Start Host — recommended: launcher script (bus topology + cameras via flags; same as teleop/record/eval scripts)
#   2-bus, no cameras                  python3 examples/humanaopen_host_launcher.py --no-cameras
#   2-bus, with cameras (default)      python3 examples/humanaopen_host_launcher.py
#   3-bus, with cameras                python3 examples/humanaopen_host_launcher.py --robot.port3 /dev/ttyACM2
#   3-bus, no cameras                  python3 examples/humanaopen_host_launcher.py --robot.port3 /dev/ttyACM2 --no-cameras
#   Override a camera device           python3 examples/humanaopen_host_launcher.py --head-camera /dev/video1
#   (--robot.port3 None = 2-bus; any device name = 3-bus. Defaults: head=/dev/video0, left_wrist=/dev/video2, right_wrist=/dev/video4)
#
# Equivalent inline form (also works):
python3 -c "
from lerobot_robot_humanaopen.humanaopen_host import HumanaOpenHost
from lerobot_robot_humanaopen import HumanaOpenConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

HumanaOpenHost(HumanaOpenConfig(
    port1='/dev/ttyACM0', port2='/dev/ttyACM1', port3=None,
    cameras={
        'head':         OpenCVCameraConfig(index_or_path='/dev/video0', fps=30, width=640, height=480, fourcc='MJPG'),  # adjust /dev/videoN to match your board (lerobot-find-cameras)
        'left_wrist':   OpenCVCameraConfig(index_or_path='/dev/video2', fps=30, width=640, height=480, fourcc='MJPG'),  # adjust /dev/videoN to match your board (lerobot-find-cameras)
        'right_wrist':  OpenCVCameraConfig(index_or_path='/dev/video4', fps=30, width=640, height=480, fourcc='MJPG'),
                # 'chest': OpenCVCameraConfig(index_or_path='/dev/video6', fps=30, width=640, height=480, fourcc='MJPG'),  # 4th camera — uncomment to enable
    },
    wheel_dir_signs={'base_left_wheel': -1, 'base_right_wheel': 1}
)).run()
"
```

> **Note**: Jetson can also run ACT inference locally (~100ms/frame on Orin),
> but SmolVLA requires a discrete GPU and should run on the dev machine.

### Dual-machine mode (--remote_ip)

All scripts (record, eval, teleop) support dual-machine mode via `--remote_ip`.
Single-machine mode (default) uses direct serial; dual-machine adds ZMQ:

```bash
# Single-machine (default — no --remote_ip needed)
python3 examples/record_data.py ...

# Dual-machine (add --remote_ip to any script)
python3 examples/record_data.py --remote_ip=192.168.1.100 --robot.cameras='{"head": {...}, "left_wrist": {...}, "right_wrist": {...}}'

python3 examples/eval_data.py --remote_ip=192.168.1.100 ...

python3 examples/teleop_leader_to_follower.py --remote_ip=192.168.1.100 ...
```

> **Note (recording)**: for `record_data.py` in dual mode, the `--remote_ip` script also
> passes the client-side camera list as a **schema** to define the dataset features.
> The Host must be started with the *same* cameras so it actually streams the images
> (start it with the camera configs, not `cameras={}`) — otherwise the recorded
> episodes contain state/action but no video.
>
> **Note (eval)**: the feature order of `observation.state` follows `robot.observation_features`
> (identical to training); `enable-base=true` keeps the `x.vel`/`theta.vel` columns.

### Network requirements

- Both machines on the same LAN (Ethernet recommended over WiFi for latency)
- Ports **5555** (commands) and **5556** (observations) must be open
- Image streaming bandwidth: ~10 Mbps per camera at 640x480 MJPG

### Performance

- **Control loop**: 60Hz (joint state read + action command)
- **Image capture**: 30fps (running in a dedicated background thread — does not block control)
- **Low latency**: image capture is decoupled from the control loop, so teleop arms respond
  instantly even at full 30fps image streaming
- **Configurable**: `image_fps_divider` (control/image rate ratio) and `max_loop_freq_hz` (control rate) in
  `HumanaOpenHostConfig`

### Wheel direction (dual-machine)

The robot's left wheel is mounted mirrored, so Host startup uses
`wheel_dir_signs={'base_left_wheel': -1, 'base_right_wheel': 1}`. This is set as the
default in `HumanaOpenConfig` — no need to pass it explicitly.

## Calibration

Calibration records the min/max range of each joint. **Only needed once** — the
results are saved and restored automatically on every connect.

### When to calibrate

- **First time setup** (required)
- After disassembling/reassembling arms or servos
- After replacing a servo motor
- After unlocking new motion range (e.g. head tilt EPROM unlock)

### Leader arm calibration

```bash
python3 examples/calibrate_leader.py
```

Steps (per arm):
1. Arm hanging straight down + gripper closed → `ENTER` (set zero point)
2. Move each joint through full range → `ENTER` (record real limits)
3. Gripper: close fully → `ENTER`, open fully → `ENTER`
4. Calibration saved automatically

> Requires leader arms powered at **7.4V** on `/dev/ttyACM2` (left) and `/dev/ttyACM3` (right).

Saved to:
```
~/.cache/huggingface/lerobot/calibration/teleoperators/humanaopen_leader/
├── leader_left.json
└── leader_right.json
```

### Follower calibration (arms + head + wheels + lift)

```bash
python3 examples/calibrate_follower.py
```

Steps:
1. Left arm + head: zero pose → `ENTER`; move joints through full range → `ENTER`
2. Right arm: zero pose → `ENTER`; move joints through full range → `ENTER`
3. Auto: wheels full range + lift stall homing to bottom

> Requires follower at **12V**. Torque is released during calibration — arms move freely.

Saved to:
```
~/.cache/huggingface/lerobot/calibration/robots/humanaopen/follower.json
```

## Teleoperation

### Full-body teleop (leader arms + keyboard)

`teleop_leader_to_follower.py` drives the follower's arms from the leader arms, and
head/base/lift from the keyboard:

| Control | Keys |
|---------|------|
| Arms | leader arms follow (flips disabled — verified same-direction) |
| Head | `w`/`s` nod (up/down), `a`/`d` shake (left/right) |
| Base | `i`/`k` forward/back, `j`/`l` turn, `n`/`m` speed (0.3x/0.6x/1.0x) |
| Lift | `u`/`h` up/down (clamped 3–200mm) |
| Quit | `b` or Ctrl+C |

```bash
# Default: 3 cameras (head + left_wrist + right_wrist)
python3 examples/teleop_leader_to_follower.py

# Add the 4th chest camera
python3 examples/teleop_leader_to_follower.py --chest-camera /dev/video6

# Dual-machine: run Host on the robot, teleop on your PC
python3 examples/teleop_leader_to_follower.py --remote_ip=192.168.1.100 --chest-camera /dev/video6

# Teleop only, no cameras
python3 examples/teleop_leader_to_follower.py --no-cameras
```

##### Bus topology: 2-bus / 3-bus switch (`--robot.port3`)

The follower's serial buses are selected on the command line, so you can move
between layouts without editing code:

| Flag | Layout | Servos on each bus |
|------|--------|--------------------|
| *(omit)* / `--robot.port3 None` | **2-bus** (default) | bus1 = left arm + head, bus2 = right arm + **lift + wheels** |
| `--robot.port3 /dev/ttyACM2` | **3-bus** | bus1 = left arm + head, bus2 = right arm (exclusive), bus3 = lift + wheels |

The 3-bus layout takes the right arm off the bus shared with the lift/wheels,
giving it an uncontended bus (cleaner 60 Hz frame timing, less move-start
latency — most noticeable on the right arm). Both layouts run the same command;
`None` (or an empty value) selects 2-bus, a device name selects 3-bus:

```bash
# 2-bus (default): lift + wheels share port2 with the right arm
python3 examples/teleop_leader_to_follower.py

# Same thing, explicit
python3 examples/teleop_leader_to_follower.py --robot.port3 None

# 3-bus: give the lift + wheels their own serial port
python3 examples/teleop_leader_to_follower.py --robot.port3 /dev/ttyACM2

# Override other ports if your device names differ
python3 examples/teleop_leader_to_follower.py --robot.port1 /dev/ttyACM3 --robot.port2 /dev/ttyACM4
```

> The head servos (IDs 12, 13) stay on bus1 in both layouts — they are
> high-frequency (written every frame) like the left arm, while the lift/wheels
> are low-frequency (read every 10th frame), so bus3's job is to isolate the
> low-frequency group. Wiring note: the new port needs the same **GND
> common-ground** with the servo power supply.

#### Live display modes

`teleop_leader_to_follower.py` supports two visualization backends. Images are
decoded once by the control loop, then handed to a background display thread
so the 60 Hz control loop is never blocked.

| Flag | Backend | Notes |
|------|---------|-------|
| `--display=rerun` | [Rerun](https://rerun.io) native viewer | Native Rerun viewer window. |
| `--display=foxglove` | [Foxglove Studio](https://foxglove.dev) WebSocket | Connects Foxglove desktop or foxglove.dev to `ws://127.0.0.1:8765`. A layout is written to `examples/humanoopen_foxglove.layout.json` (import once). **Recommended** — lowest render latency of the two. |

Omit `--display` to run headless (no viewer).

```bash
# Foxglove (recommended)
python3 examples/teleop_leader_to_follower.py --remote_ip=192.168.1.100 --display=foxglove

# Rerun
python3 examples/teleop_leader_to_follower.py --remote_ip=192.168.1.100 --display=rerun
```

> **Display latency**: Both backends add a fixed render pipeline delay
> (≈1–1.5 s desktop / ≈2 s native viewer on this hardware) which is
> inherent to the visualization libraries. The action channel (arm
> commands) is real-time regardless; display lag does **not** affect
> recorded data quality — observation and action frames are always
> sampled synchronously.

Camera arguments: `--cameras=head,left_wrist` (subset), `--head-camera /dev/videoN`,
`--left-wrist-camera`, `--right-wrist-camera`, `--chest-camera` (each overrides the
device path; passing a `--*-camera` arg auto-adds that camera).

### Camera devices & fps (tested)

| Camera | Device | Format | FPS |
|--------|--------|--------|-----|
| head | /dev/video0 | MJPG | 30 |
| left_wrist | /dev/video2 | MJPG | 30 |
| right_wrist | /dev/video4 | MJPG | 30 |
| chest | /dev/video6 | MJPG | 30 |

> **Adjusting FPS**: Check your camera's actual capabilities with
> `v4l2-ctl -d /dev/videoN --list-formats-ext`, then update the `fps` value in scripts.
> Setting an unsupported fps will cause a connection error at startup.



### Lift axis — zero persistence (免归零)

The lift is a 12-bit single-turn encoder (4096 ticks/rev) driving a leadscrew
(25 revs = 200mm). Absolute position is tracked in software via multi-turn wrap
tracking. Because the leadscrew is self-locking, the mechanical position survives
power cycles — so the zero position is persisted to `~/.cache/humanaopen/lift_zero.json`
and restored on the next connect, **skipping re-homing**:

- First connect: homes to the bottom (stall detection), saves the zero.
- Later connects: restores the saved absolute position (no movement needed).
- If the position changed (e.g. the lift was moved manually), restore fails and
  auto-homing runs instead.

Lift tuning (tested): `v_max=110` (raw), `kp_vel=10`, `home_down_speed=10` with
Phase BIT2=0 (50 step/s per raw unit). Max speed ≈ 8.7mm/s (200mm in ~23s).

### Lift speed boost (BIT2=0)

The ST3250 firmware maps `Goal_Velocity` with Phase BIT2=1 at 1 step/s per raw unit,
where raw > 1000 wraps direction (triangular wave) — unsafe. Switching Phase BIT2=0
changes the unit to 50 step/s per raw, so full speed (5500 step/s) is just raw 110,
entirely inside the reliable range. **After switching, all velocity params must be
divided by 50** (`home_down_speed`, `kp_vel`, `v_max`). Tools:
`examples/switch_phase_bit2.py` (toggle), `examples/speed_test_bit2_0.py` (verify).

> **Modern code does this automatically**: `lift_axis.configure()` reads the Phase
> register on every startup and, if BIT2=1 is detected, rewrites it to BIT2=0 (EPROM
> write — persists across reboots). `Acceleration` is also forced to 254 (fastest
> ramp) so commanded speeds are actually reached. No manual switching needed.

**After replacing the lift servo (safest manual fallback)**: a new servo ships with
Phase BIT2=1 (factory default), and the old `~/.cache/humanaopen/lift_zero.json`
holds the previous motor's encoder data — both need handling.

```bash
# 1. Remove the stale zero file (new motor has a different encoder origin)
rm ~/.cache/humanaopen/lift_zero.json

# 2. Check / switch the velocity unit (new servo ships BIT2=1 → set BIT2=0)
python3 examples/check_phase.py          # report current BIT2
python3 examples/switch_phase_bit2.py    # switch BIT2 → 0

# 3. First boot with forced homing to write the new motor's true zero
python3 -c "
from lerobot_robot_humanaopen.humanaopen_host import HumanaOpenHost
from lerobot_robot_humanaopen import HumanaOpenConfig
HumanaOpenHost(HumanaOpenConfig(
    port1='/dev/ttyACM0', port2='/dev/ttyACM1', port3=None,
    cameras={}, force_lift_home=True
)).run()
"
```

> Normal startup already performs steps 2–3 automatically; running them manually is
> a belt-and-braces check after a motor swap before going back to the full flow.

### Head tilt range unlock

The head tilt servo had its EPROM position limits baked to [1430, 2096] (~58°),
which the calibration file copied — limiting tilt to -54°/+4°. Writing
`Min=0 / Max=4095` unlocks the mechanical range [1367, 2242] (-61.6°/+17.1°):
`examples/unlock_head_tilt.py --probe`. After unlocking, the calibration file
(`~/.cache/huggingface/lerobot/calibration/robots/humanaopen/follower.json`) was
updated to the real range.

## Data Collection

The `lerobot-record` CLI hardcodes official robot types and rejects `humanaopen`
as an unrecognized choice. Use the Python API wrapper `examples/record_data.py`
instead — it exposes the **same parameter names** as `lerobot-record` and prints
the equivalent CLI command on startup for reference.

### 3 cameras (default)

```bash
python3 examples/record_data.py \
    --robot.type=humanaopen \
    --robot.id=follower \
    --robot.port1=/dev/ttyACM0 \
    --robot.port2=/dev/ttyACM1 \
    --robot.port3=None \
    --robot.cameras='{"head": {"type": "opencv", "index_or_path": "/dev/video0", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "left_wrist": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "right_wrist": {"type": "opencv", "index_or_path": "/dev/video4", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}}' \
    --robot.confirm_lift_after_home=true \
    --teleop.type=humanaopen_teleop \
    --teleop.left_arm_port=/dev/ttyACM2 \
    --teleop.right_arm_port=/dev/ttyACM3 \
    --teleop.flip_joints='{"left": [], "right": []}' \
    --teleop.joint_remap='{}' \
    --dataset.repo_id=your-name/humanaopen_demo \
    --dataset.single_task="describe your task" \
    --dataset.num_episodes=2 \
    --dataset.episode_time_s=15 \
    --dataset.reset_time_s=10 \
    --dataset.fps=30 \
    --dataset.push_to_hub=true
```

### 4 cameras (with chest for navigation)

Same as above, but replace the `--robot.cameras` JSON to include chest:

```bash
    --robot.cameras='{"head": {"type": "opencv", "index_or_path": "/dev/video0", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "left_wrist": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "right_wrist": {"type": "opencv", "index_or_path": "/dev/video4", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "chest": {"type": "opencv", "index_or_path": "/dev/video6", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}}' \
```

> **Note**: camera names must be consistent across record / train / rollout.

### Dual-machine (ZMQ): Host on the robot, record on your PC

When the follower runs on the robot (with its own cameras) and the leader arms
are plugged into your PC, record over ZMQ. **Start the Host first**, then run
`record_data.py` on the PC:

```bash
# 1) On the robot (Jetson or Raspberry Pi, see "Installing: Raspberry Pi / Jetson"
#    above) — start the Host, then leave it running
#    (HumanaOpenHost(...).run() with port1/port2 + the 3 cameras).

# 2) On your PC — record (follower via ZMQ, leader via local serial ttyACM0/1)
python3 examples/record_data.py \
    --remote_ip=192.168.1.9 \
    --robot.cameras='{"head": {"type": "opencv", "index_or_path": "/dev/video0", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "left_wrist": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "right_wrist": {"type": "opencv", "index_or_path": "/dev/video4", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}}' \
    --teleop.left_arm_port=/dev/ttyACM0 \
    --teleop.right_arm_port=/dev/ttyACM1 \
    --teleop.flip_joints='{"left": [], "right": []}' \
    --teleop.joint_remap='{}' \
    --dataset.repo_id=your-name/humanaopen_demo \
    --dataset.single_task="describe your task" \
    --dataset.num_episodes=2 \
    --dataset.episode_time_s=15 \
    --dataset.reset_time_s=10 \
    --dataset.fps=30 \
    --dataset.push_to_hub=true \
    --display=foxglove
```

- `--remote_ip` switches to dual-machine mode: the follower connects to the Host
  over ZMQ (follower `port1/port2`, `--robot.type/id/port3` are **not** needed),
  while the leader arms use the PC's local serial ports.
- `--robot.cameras` is passed as a *schema* (names + resolutions); the Host owns
  the physical cameras and streams the images over ZMQ.
- `--display=foxglove` (optional) streams cameras + state to the Foxglove app on
  the PC (connect Studio to `ws://127.0.0.1:8765`); `--display=rerun` uses the
  native Rerun viewer. Omit `--display` to record headless.

### Controls during recording

| Control | Keys |
|---------|------|
| Arms | leader arms follow (16 DOF) |
| Head | `w`/`s` nod, `a`/`d` shake (2 DOF) |
| Base | `i`/`k` forward/back, `j`/`l` turn (2 DOF, speed `n`/`m`) |
| Lift | `u`/`h` up/down with safety limits (1 DOF, clamped 3–200mm) |
| Record | `C` start, `Q` quit, `A` re-record episode |
| Confirm | After homing, hold `u`/`h` to position, `ENTER` to confirm |

The `--teleop.type=humanaopen_teleop` teleoperator records **all 21 DOF** — the
leader arms (16 joints) plus keyboard-controlled head/lift/base (5 DOF). Both are
saved into the dataset for ACT training.

### Lift behavior during recording

- On first connect: lift **homes to bottom** (stall detection), saves the zero
  position to `~/.cache/humanaopen/lift_zero.json`.
- On subsequent connects: lift **restores the saved position** (no homing needed),
  unless the position changed (manual push → restore fails → auto-home).
- After homing: keyboard hold `u`/`h` adjusts height with safety limits
  (3mm–200mm), `ENTER` to confirm and start recording.

### Resuming / cleaning datasets

If a dataset directory already exists from a previous run, delete it or resume:

```bash
rm -rf ~/.cache/huggingface/lerobot/your-name/humanaopen_demo    # fresh start
# or add --dataset.resume=true to the record_data.py command     # continue from last episode
```

### Live display while recording (optional)

By default recording runs headless (no viewer). To watch cameras + state in real
time while you teleoperate, add a display flag:

```bash
# Rerun viewer
python3 examples/record_data.py ... --display=rerun

# Foxglove app (recommended — lower render latency, same backend as teleop)
python3 examples/record_data.py ... --display=foxglove
# connect Foxglove Studio to ws://127.0.0.1:8765
```

> Display is decoupled from the control/recording loop, so it never perturbs the
> recorded `(observation, action)` frames. Like teleop, the viewer adds a fixed
> ~1–1.5 s render delay that does **not** affect data quality.

## Training

### ACT (action chunking transformer)

```bash
# Quick test (2 episodes)
lerobot-train \
    --policy.type=act \
    --policy.device=cuda \
    --policy.push_to_hub=true \
    --policy.repo_id=your-name/humanaopen_act_policy \
    --dataset.repo_id=your-name/humanaopen_act_demo \
    --output_dir=outputs/humanaopen_act_demo \
    --batch_size=3 \
    --steps=5

# Production (>50 episodes)
lerobot-train \
    --policy.type=act \
    --policy.device=cuda \
    --policy.push_to_hub=true \
    --policy.repo_id=your-name/humanaopen_act_policy \
    --dataset.repo_id=your-name/humanaopen_act_demo \
    --output_dir=outputs/humanaopen_act_demo \
    --batch_size=32 \
    --steps=50000
```

### SmolVLA (vision-language-action model)

SmolVLA requires the language instruction from `--dataset.single_task` (used during recording).
The VLM weights (~500M) are downloaded automatically from HuggingFace on first run.

```bash
# Quick test (2 episodes)
lerobot-train \
    --policy.type=smolvla \
    --policy.device=cuda \
    --policy.push_to_hub=true \
    --policy.repo_id=your-name/humanaopen_smolvla_policy \
    --dataset.repo_id=your-name/humanaopen_act_demo \
    --output_dir=outputs/humanaopen_smolvla_demo \
    --batch_size=4 \
    --steps=20
```

> **Note**: SmolVLA (~450M params) is ~20x heavier than ACT (~52M), uses more VRAM,
> and trains slower. Batch size 4 fits in 8GB VRAM (RTX 5060 Ti). For >50 episodes,
> increase steps to 20000+.

### Key parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--policy.type` | — | **Required.** `act`, `smolvla`, `diffusion`, etc. |
| `--policy.device` | `cuda` | `cuda` / `cpu`. |
| `--policy.push_to_hub` | `true` | Push model to HuggingFace Hub. |
| `--policy.repo_id` | — | Hub repo for the trained model. Required when pushing. |
| `--dataset.repo_id` | — | **Required.** Hub repo of the training dataset. |
| `--output_dir` | — | Local checkpoint directory. |
| `--batch_size` | 8 | Samples per step. ACT: 32, SmolVLA: 4 (8GB VRAM limit). |
| `--steps` | 100000 | Total training steps. ACT: 50K, SmolVLA: 20K. |

### Outputs

```
outputs/humanaopen_act_demo/
├── pretrained_model/           # Full model (config + weights)
├── last/pretrained_model       # Latest checkpoint
├── train_logs/                 # Training metrics (TensorBoard-compatible)
└── training_state.json         # Optimizer/scheduler state for resume
```

The pushed model will be at `https://huggingface.co/your-name/humanaopen_act_policy`.

## Inference (Deployment)

> **Dependencies**: SmolVLA requires `transformers>=4.48` and `num2words`.
> Install with `pip install transformers>=4.48 num2words` before running SmolVLA inference.

### ACT inference

**Dual-machine (recommended):** the robot runs a Host (Jetson/Raspberry Pi) with the
cameras; `eval_data.py` connects to it over ZMQ on your PC. Start the Host first, then:

```bash
# On the robot (Jetson or Raspberry Pi) — start the Host and leave it running
#   (HumanaOpenHost(...).run() with port1/port2 + the 3 cameras).

# On your PC — run inference
python3 examples/eval_data.py \
    --policy.type=act \
    --policy.repo_id=your-name/humanaopen_act_policy \
    --policy.device=cuda \
    --remote_ip=192.168.1.9 \
    --robot.cameras='{"head": {"type": "opencv", "index_or_path": "/dev/video0", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "left_wrist": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "right_wrist": {"type": "opencv", "index_or_path": "/dev/video4", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}}' \
    --enable-base=false \
    --enable-lift=false \
    --num-episodes=5 \
    --duration=30 \
    --fps=30 \
    --display=foxglove
```

- `--remote_ip` switches to dual-machine mode: the follower connects to the Host
  over ZMQ (no `--robot.port1/port2/port3`); the cameras live on the Host.
- `--display=foxglove` (optional) streams to the Foxglove app on the PC
  (connect Studio to `ws://127.0.0.1:8765`). Omit `--display` for headless inference.

**Single-machine** (follower directly on serial ports, same machine):
```bash
python3 examples/eval_data.py \
    --policy.type=act \
    --policy.repo_id=your-name/humanaopen_act_policy \
    --policy.device=cuda \
    --robot.type=humanaopen \
    --robot.id=follower \
    --robot.port1=/dev/ttyACM0 \
    --robot.port2=/dev/ttyACM1 \
    --robot.port3=None \
    --robot.cameras='{"head": {"type": "opencv", "index_or_path": "/dev/video0", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "left_wrist": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "right_wrist": {"type": "opencv", "index_or_path": "/dev/video4", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}}' \
    --enable-base=false \
    --enable-lift=false \
    --num-episodes=5 \
    --duration=30 \
    --fps=30
```

### SmolVLA inference (language-conditioned)

**Dual-machine (recommended):** same as ACT — start the Host on the robot, run on
your PC over ZMQ:

```bash
# On the robot — start the Host and leave it running.

# On your PC
python3 examples/eval_data.py \
    --policy.type=smolvla \
    --policy.repo_id=your-name/humanaopen_smolvla_policy \
    --policy.device=cuda \
    --task="wave hello with both arms" \
    --remote_ip=192.168.1.9 \
    --robot.cameras='{"head": {"type": "opencv", "index_or_path": "/dev/video0", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "left_wrist": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "right_wrist": {"type": "opencv", "index_or_path": "/dev/video4", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}}' \
    --enable-base=false \
    --enable-lift=false \
    --num-episodes=2 \
    --duration=10 \
    --fps=10 \
    --display=foxglove
```

**Single-machine** (follower directly on serial):
```bash
python3 examples/eval_data.py \
    --policy.type=smolvla \
    --policy.repo_id=your-name/humanaopen_smolvla_policy \
    --policy.device=cuda \
    --task="wave hello with both arms" \
    --robot.type=humanaopen \
    --robot.id=follower \
    --robot.port1=/dev/ttyACM0 \
    --robot.port2=/dev/ttyACM1 \
    --robot.port3=None \
    --robot.cameras='{"head": {"type": "opencv", "index_or_path": "/dev/video0", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "left_wrist": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "right_wrist": {"type": "opencv", "index_or_path": "/dev/video4", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}}' \
    --enable-base=false \
    --enable-lift=false \
    --num-episodes=2 \
    --duration=10 \
    --fps=10
```

> **SmolVLA performance note**: VLM inference is ~1s/frame (450M params). A 10s
> episode at 10fps = 100 frames ≈ 100s wall clock time. For real-time deployment,
> use ACT (~50ms/frame). SmolVLA is best for language-conditioned tasks.

### Live display during inference (optional)

Display is off by default. Add `--display` to stream the rollout live:

```bash
# Foxglove app (recommended — lower render latency)
python3 examples/eval_data.py ... --display=foxglove
# connect Foxglove Studio to ws://127.0.0.1:8765

# Rerun native viewer
python3 examples/eval_data.py ... --display=rerun

# Headless (default): omit --display
python3 examples/eval_data.py ...
```

> Display runs in a background thread so it never slows the policy loop. Like
> teleop/record, the ~1–1.5 s render delay is inherent to the viewer and does not
> affect control.

### Base control (disable by default)

By default the base wheels are **disabled** (`--enable-base=false`): the policy's
predicted `x.vel` / `theta.vel` are forced to `0`, so only the arms, head and lift
move. This prevents the robot from driving away unexpectedly, which is safer while
validating a new policy.

```bash
# Base disabled (default, safer) — wheels stay still, only arms/head/lift move
python3 examples/eval_data.py ... --enable-base=false

# Base enabled — policy is allowed to drive the wheels
python3 examples/eval_data.py ... --enable-base=true
```

Start with `--enable-base=false` until the policy behaves well, then enable the
base if your task requires locomotion.

### Lift control (hold height by default)

By default the lift is **held at its current height** (`--enable-lift=false`): the
policy's `lift_axis.height_mm` is overridden to the current height each frame, so
the lift does not move (the policy can otherwise output 0 and drag the lift to the
bottom).

```bash
# Lift held at current height (default, safer) — the lift stays put
python3 examples/eval_data.py ... --enable-lift=false

# Lift enabled — policy controls the lift height
python3 examples/eval_data.py ... --enable-lift=true
```

Start with the lift held, then enable it once your task requires changing height.

### Controls during inference

| Control | Keys |
|---------|------|
| Quit | `q` or Ctrl+C |

### Safety: torque release on exit

When you press `q` / Ctrl+C to stop inference, the script will:
1. **Stop the base wheels** (send `x.vel=0 / theta.vel=0` as the final command) and
   hold the lift at its current height
2. Save the lift position
3. Prompt: **"Press ENTER to release torque and disconnect..."**
4. Hold the arms before pressing ENTER — torque will be released and arms drop freely
5. Press ENTER when ready → disconnect

This prevents arms from suddenly dropping when servos lose power.

**Base/lift auto-stop safety net (dual-machine):** the Host runs a watchdog — if it
receives no action command for `watchdog_timeout_ms` (500 ms), it zeroes the wheel
velocity and holds the lift. So even if the PC process is killed abruptly
(`kill -9`) or disconnects while the wheels are moving, the base stops on its own
instead of continuing to drive until the Host is stopped.

### Key parameters (inference)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--policy.type` | `act` | `act` or `smolvla`. |
| `--policy.repo_id` | — | **Required.** Hub repo of the trained model. |
| `--task` | — | Language instruction (required for SmolVLA). |
| `--enable-base` | `false` | Set `true` to allow policy control of base wheels. Default: disabled. |
| `--enable-lift` | `false` | Set `true` to let the policy control the lift height. Default: held at current height. |
| `--num-episodes` | 5 | Number of inference episodes. |
| `--duration` | 30 | Seconds per episode. |
| `--fps` | 30 | Inference frequency (Hz). |

On startup, the script will prompt for calibration confirmation (ENTER to restore).
On exit, it will prompt for torque release confirmation (ENTER to release).

## Navigation

Language-driven navigation powered by [LightNav-0](https://github.com/kyonofx/LightNav-0) — a vision-language-action model running on your GPU PC. The robot only runs a light ZMQ adapter; no LightNav install, no ROS, no lidar, no map needed. Just a forward RGB camera plus the GPU server over the network.

```
[chest camera /dev/video0] ──cv2──> [adapter on Jetson] ──WS──> lightnav-serve (GPU PC, :8050)
        ^                                   |
        |                                   | 10×SE(2) waypoints / step
  HumanaOpenHost <──ZMQ :5555── [control law: official waypoint_command]
```

Quick start (on the robot, two terminals):

```bash
# Terminal 1 — Host in pure-base mode (no camera streaming; adapter reads the chest cam itself)
python3 -c "
from lerobot_robot_humanaopen.humanaopen_host import HumanaOpenHost
from lerobot_robot_humanaopen import HumanaOpenConfig

HumanaOpenHost(HumanaOpenConfig(
    port1='/dev/ttyACM0', port2='/dev/ttyACM1', port3=None,
    cameras={},
    wheel_dir_signs={'base_left_wheel': -1, 'base_right_wheel': 1}
)).run()
"

# Terminal 2 — navigation adapter (GPU PC must have lightnav-serve listening on 0.0.0.0:8050)
NO_PROXY="192.168.1.12" python3 examples/lightnav_navigation.py \
    --lightnav ws://192.168.1.12:8050 \
    --camera /dev/video0 \
    --instruction "walk forward slowly"
```

Key points:
- **English instructions only** (model is English-trained); e.g. `"walk forward slowly"`, `"go to the object on your left"`.
- `NO_PROXY=<gpu_ip>` is required on the robot: the `websockets` client reads proxy env vars and will fail to reach the LAN GPU server otherwise.
- RVQ quantization traps (waypoint yaw = ±0.314 rad for "no turn", 0.15 m steps) are handled inside the adapter via the official `waypoint_command` control law + deadbands — see `docs/navigation/lightnav_integration.md` §八/§九 for the full pitfall log.

## Roadmap

| Module | Status |
|--------|--------|
| ACT, SmolVLA | ✅ |
| BOM | ✅ |
| Assembly | ✅ |
| CAD | ✅ |
| Lite version | ☐ |
| URDF | ☐ |
| Agent | ☐ |
| Vision-based grasping | ☐ |
| Vision-based navigation | ✅ |
| Embodied world models | ☐ |

### Acknowledgements

- Software is built on [LeRobot](https://github.com/huggingface/lerobot) and
  [open-arms-mini](https://github.com/TheRobotStudio/open-arms-mini).
- The arms are based on open-arms-mini, with a [PincOpen](https://github.com/pollen-robotics/PincOpen?tab=readme-ov-file) gripper added at the end of each arm.
- Hardware design references [xlerobot](https://github.com/xrobot/xlerobot):
  the head degrees-of-freedom and the differential-drive base follow its
  approach.

## Contact & Cooperation

- **Product Communication**: scan the QR code to reach our product team:

<img src="docs/Product_Communication" alt="Product communication QR code" width="25%">

## License

Apache 2.0
