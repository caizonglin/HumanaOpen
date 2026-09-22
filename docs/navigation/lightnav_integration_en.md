# HumanaOpen Navigation — LightNav-0 Integration: Research & Implementation Plan

> Status: **Research complete, simulation verified, real-robot integration done (straight-line + turning tests passed)**
> Date: 2026-09-17 (research) / 2026-09-18 (real-robot integration)
> Hardware: PC (RTX 5060 Ti 16GB) as the GPU inference server, Jetson as the robot side

## 1. Executive Summary

- **LightNav-0 is feasible**: environment setup, model loading, and full MuJoCo-simulated language navigation all verified on the PC (ALL PASS).
- **Zero changes to HumanaOpen code**: navigation is "attached" as an external module through a ~50-line ZMQ adapter that interfaces with the HumanaOpen chassis.
- **No ROS layer needed** (minimal plan); the full ROS 2 stack is an optional upgrade, not a prerequisite.
- **Minimal sensor needs**: only one forward-facing RGB camera (no lidar, no depth, no map, no odometry).

## 2. What LightNav-0 Is

- **A VLM navigation model** (built on Qwen3-VL-4B): takes monocular RGB history frames + natural-language instruction → outputs 10 SE(2) future waypoints `[forward_m, lateral_m, yaw_rad]`.
- **Not SLAM / move_base / a mapping framework**: no localization, no mapping, no obstacle-perception layer; pure end-to-end vision+language → waypoints, replanned every frame.
- One model covers: language-instruction navigation (VLN), open-vocabulary object search (ObjectNav), visual target following (EVT), and zero-shot cross-robot transfer (officially demoed on humanoid / quadruped / wheeled / aerial).
- Project page / source: `LightOrigins/LightNav-0`, Apache 2.0, arXiv 2608.30935.
- Model weights: HuggingFace `LightOriginsHQ/LightNav-0` (requires HF login; in-repo files ~9.6GB).

## 3. Hardware & Runtime Requirements

| Item | Requirement | Measured (this 5060 Ti) |
|---|---|---|
| GPU | ~15GB VRAM (4B bf16 ~10GB + KV cache ~2.3GB) | ✅ 16GB available, ~8.7GiB used |
| Arch | officially validated sm_103 (B300); consumer Blackwell (sm_120) also tested working | ✅ |
| Python | >=3.11, <3.12 | ✅ dedicated conda env `lightnav` |
| torch | >=2.9; **Blackwell needs the cu129 wheel** (2.10.0+cu129) | ✅ |
| vLLM | **pinned exactly 0.19.1** (binds to private APIs) | ✅ |
| Inference latency | official 60-150ms/step; this machine 644ms/step (after warmup) | usable |

Sensor: a single monocular RGB forward camera (reference: Orbbec Gemini 330, 640×360@30; any rgb8 camera works).
Robot: any mobile chassis; official references Unitree Go2 / LimX TRON1; the **MuJoCo demo runs a differential-drive TurtleBot (same kinematics as HumanaOpen)**.

## 4. Deployment Architecture (Relationship to HumanaOpen)

```
┌─ LightNav world ────────────────┐      ┌─ HumanaOpen world ──────────────┐
│ GPU machine (PC)                │      │ Jetson robot side               │
│  lightnav-serve :8050           │      │  HumanaOpenHost (ZMQ 5555/5556) │
│  ←WebSocket→                    │      │   chassis consumes x.vel/theta.vel │
│        │                        │      │                                │
│        └─ waypoint → ZMQ adapter ────▶│   differential motors           │
│    (adapter ~50 lines)          │      │                                │
└─────────────────────────────────┘      └────────────────────────────────┘
```

- **Model server**: runs on the GPU machine (PC) with `lightnav-serve`, in its own conda env `lightnav`.
- **Client (adapter)**: on the Jetson, camera → JPEG → WebSocket `next` → get waypoints → convert to `v=fwd/dt, w=yaw/dt` → ZMQ to HumanaOpenHost's `x.vel/theta.vel`.
- **HumanaOpen code unchanged**: the adapter only feeds values into the existing ZMQ action interface.

### Optional: full ROS 2 stack (not adopted)
The official `robot_deploy/` is a ROS 2 Humble stack (camera driver + vln_client + vln_mpc + vln_web + adapters) offering MPC smoothing, web control panel, watchdog, and manual/auto switching. **It needs ROS 2 installed + writing a humano_adapter; that's heavy — deferred.**

## 5. Install Steps (done on the PC, archived)

```bash
# 1. dedicated Python 3.11 env
conda create -n lightnav python=3.11 -y
conda activate lightnav

# 2. Blackwell requires cu129 torch (plain cu12.8 can't JIT)
pip install torch==2.10.0+cu129 torchvision==0.25.0+cu129 \
  --index-url https://download.pytorch.org/whl/cu129

# 3. LightNav itself
cd ~/LightNav-0
pip install -e ".[vllm,video]"

# 4. HF login (need socksio first in a proxied env)
pip install "httpx[socks]"
hf auth login --token <TOKEN> --add-to-git-credential

# 5. download weights
mkdir -p checkpoints/LightNav-0
hf download LightOriginsHQ/LightNav-0 --local-dir checkpoints/LightNav-0

# 6. GPU smoke test (continue only if ALL PASS)
MODEL_PATH=checkpoints/LightNav-0 bash scripts/smoke_gpu.sh
```

## 6. Starting the Model Server (persistent on the GPU machine)

```bash
cd ~/LightNav-0 && conda activate lightnav
CUDA_VISIBLE_DEVICES=0 GPU_MEM_UTIL=0.78 \
HOST=0.0.0.0 PORT=8050 lightnav-serve \
    --task vln \
    --model_path checkpoints/LightNav-0 \
    --backend vllm_local
```

- Listen on `127.0.0.1:8050` by default (local). **For the Jetson to connect, it must listen on an external address** — confirm `lightnav-serve`'s host param before starting (or set `--host 0.0.0.0`).
- First start includes model load + JIT compile (~1-2 min); per-step inference afterward ~100-644ms.
- If VRAM is tight, lower `GPU_MEM_UTIL` to 0.70.

## 7. MuJoCo Simulation Verification (passed)

```bash
# keep the service terminal running; open another terminal:
cd ~/LightNav-0/mujoco_demo
./run.sh --vln-server ws://127.0.0.1:8050
# open http://127.0.0.1:8088 in a browser and enter English instructions (e.g. "walk to the red trash can")
```

- Simulated differential TurtleBot (same kinematics as HumanaOpen) does visual language navigation in a ProcTHOR indoor scene.
- The model is trained in English; **Chinese instructions are not supported — use English**.

## 8. Integrating with HumanaOpen (implemented)

### ZMQ adapter (actual implementation: `examples/lightnav_navigation.py`)

Runs on the Jetson (Jetson only needs `pip install websockets`; no LightNav install or GPU inference — the model runs entirely on the PC):

```bash
cd ~/HumanaOpen && git pull
# start HumanaOpenHost (chassis-only; the adapter grabs the camera itself)
python3 -c "
from lerobot_robot_humanaopen.humanaopen_host import HumanaOpenHost
from lerobot_robot_humanaopen import HumanaOpenConfig

HumanaOpenHost(HumanaOpenConfig(
    port1='/dev/ttyACM0', port2='/dev/ttyACM1', port3=None,
    cameras={},   # navigation doesn't need host image relay; the adapter captures the chest camera itself
    wheel_dir_signs={'base_left_wheel': -1, 'base_right_wheel': 1}
)).run()
"
# another terminal: run the navigation adapter
NO_PROXY="192.168.1.12" python3 examples/lightnav_navigation.py \
    --lightnav ws://192.168.1.12:8050 \
    --zmq-host 127.0.0.1 \
    --camera /dev/video0 \
    --instruction "walk forward slowly"
```

Workflow:
1. `cv2` captures one frame from the chest camera → JPEG (q80, 640x360) → WebSocket sent to the PC's lightnav-serve
2. The service returns 10 SE(2) waypoints `[forward_m, lateral_m, yaw_rad]` (RVQ-quantized)
3. **Control law** (ported from LightNav's official mujoco demo `waypoint_command`, NOT the naive "waypoint ÷ dt"):
   - use only `waypoints[0]` (the current-frame action; later waypoints are predicted trajectory — using them amplifies atan2 noise → full-speed spinning)
   - `distance < 0.08m` → treat as quantization noise, stop `(0,0)`
   - `angular = clamp(1.8·bearing + 0.25·target_yaw)`, each with a deadband to filter quantization levels
   - `linear = clamp(0.75·distance·cos(bearing), 0, 0.35)`; when `|bearing|>65°` → linear=0 (turn first, then move)
4. `{"x.vel": v m/s, "theta.vel": w deg/s}` sent via ZMQ PUSH to HumanaOpenHost:5555

Key points:
- `stop=true` or empty waypoints → send `{x.vel:0, theta.vel:0}`; Ctrl+C also actively zeroes (not only relying on the watchdog)
- **RVQ quantization pitfall** (real-robot lesson): the action tokenizer quantizes "no turn" to `yaw=±0.314` (≈±π/10) and "one step straight" to `forward≈0.15m`. Direct `yaw/dt` gives crazy ±72°/s self-spin; direct `forward/dt` is a 0.6 m/s collision speed. The official control law + deadbands above are mandatory.
- **Proxy gotcha**: the `websockets` library reads http/socks proxy env vars; on the Jetson connecting to the LAN GPU you MUST set `NO_PROXY=192.168.1.12` or it can't connect.
- **Voice/Chinese** requires an extra ASR/translation layer (the model is English-trained) — out of current scope.

### Camera
- Real robot uses the Jetson's `Integrated_Webcam_HD` (USB, native MJPG 1280x720@30 / YUYV 640x480@25) as the **chest forward camera `/dev/video0`**.
- Note: lerobot's `OpenCVCamera` can't set this camera's resolution (`set()` always returns False), so host uses `cameras={}` chassis-only mode; images are captured entirely by the adapter's `cv2` directly (`cv2.VideoCapture` on `/dev/video0` works).
- The model internally resizes any resolution to its `video_size`; the client only needs to JPEG-encode.

## 9. Real-Robot Verification Results (2026-09-18)

### Environment
- Jetson (host, `humanaopen` conda env, Python 3.12): HumanaOpenHost chassis-only + adapter; only peripheral: chest camera `/dev/video0`
- PC (GPU, `lightnav` env): lightnav-serve persistent on `0.0.0.0:8050`, model ~8.7GiB VRAM

### Key data
| Item | Measured |
|---|---|
| Per-frame inference latency (Jetson→PC→Jetson RTT) | **~200-350ms** (faster than the 644ms pure-local warmup; more stable once the service is persistent) |
| Straight `"walk forward slowly"` | **0.113 m/s steady straight**, no self-spin, no jitter |
| Left-turn arc `"go to the object on your left"` | **0.11 m/s + 15°/s left turn** (model outputs lateral=0.022m → bearing=8.4° → 1.8×=15°/s) |
| Stationary (noisy waypoints) | **exactly zero** — RVQ quantization noise (yaw=±0.314) filtered by deadband, no more ±72°/s spinning |
| Watchdog interaction | host 500ms timeout OK; adapter 4Hz control cycle within timeout |

### Pitfall log (chronological)
1. **`host+cameras` startup crash**: `OpenCVCamera(/dev/video0)` validation failed — this USB camera's `set(CAP_PROP_*)` always returns False, lerobot treats it as failure. → switched to `cameras={}` chassis-only mode.
2. **±72°/s self-spin** (first batch): direct `yaw/dt`, quantized `±0.314rad` became full-speed rotation. → introduced official `waypoint_command`.
3. **±45°/s jitter** (second batch): the official 0.35m distance threshold treated every 0.15m step as noise, falling back to `valid[-1]` which amplified atan2 sign noise. → lowered threshold to 0.08m + bearing/yaw deadbands.
4. **-45°/s residual** (third batch): when searching for the "first sufficiently-far point", far points later in the sequence had huge bearing and overrode waypoint[0]. → use only `waypoint[0]` as the current-frame action.

### Conclusion
- **Navigation pipeline fully working**: camera → GPU inference → control law → chassis, all wired up.
- The model correctly understands "left/right/forward" semantics, outputting the right lateral shift + turn combination.
- Current speeds conservative (0.11 m/s, 15°/s); tune via `--max-linear` / `--max-angular`.

## 10. Risks & Notes

| Item | Note |
|---|---|
| **No obstacle avoidance** | Model only sees RGB, can't sense obstacles outside view; emergency stop relies on the `stop` flag + user WASD/manual control |
| **Latency** | Measured ~200-350ms per step (~3-5Hz replanning); for far movement use a smoother, don't step-shoot |
| **Chinese instructions** | Model is English-trained; Chinese needs a translation/ASR layer first |
| **Service external-facing** | lightnav-serve MUST listen on `0.0.0.0` for Jetson to reach it (`HOST=0.0.0.0` env var) |
| **Proxy** | Jetson connecting to `ws://192.168.1.12:8050` needs `NO_PROXY=192.168.1.12` (websockets reads proxy vars) |
| **VRAM** | 16GB is a bit tight; close other GPU-heavy programs while serving |
| **Change servo/env** | No effect on navigation (navigation only depends on camera + GPU) |

## File Structure

```
docs/navigation/
└── lightnav_integration_en.md   # this doc (English, HumanaOpen side)
HumanaOpen-side implementation:
examples/lightnav_navigation.py  # ZMQ adapter: camera capture → ws → control law → chassis
External:
~/LightNav-0/                    # LightNav repo (separate project, not merged into HumanaOpen)
  ├── src/lightnav/              # model inference + WebSocket service
  ├── robot_deploy/              # (optional) ROS2 reference stack
  ├── mujoco_demo/               # differential-robot simulation (validated)
  └── docs/                      # PROTOCOL/DEPLOYMENT/CONFIGURATION
```

## 11. Roadmap

- [x] Env setup + GPU smoke test (ALL PASS)
- [x] Model service starts
- [x] MuJoCo simulated language navigation validated
- [x] Confirm Jetson forward camera (chest `/dev/video0`, `Integrated_Webcam_HD`)
- [x] Make service listen on 0.0.0.0
- [x] Write ZMQ adapter (on Jetson, `examples/lightnav_navigation.py`)
- [x] HumanaOpen + LightNav real-robot end-to-end test (straight + left-turn arc both pass)
- [ ] Obstacle-avoidance enhancement (current model has no obstacle avoidance, RGB semantic navigation only)
- [ ] Tune speed/steering feel (`--max-linear` / `--max-angular`)
- [ ] Goal-point stop accuracy validation (`stop` flag behavior on the real robot)
