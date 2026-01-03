---
name: landerpi-hailo
description: Hailo 8 AI accelerator integration for HiWonder LanderPi robot. Provides hardware-accelerated YOLOv11 inference at 25-40 FPS using HailoRT 4.23. Requires landerpi-core skill for connection setup. (project) (project) (project)
---

# LanderPi Hailo 8 AI Accelerator

## Overview

Hardware acceleration for YOLOv11 object detection using Hailo-8 NPU on Raspberry Pi 5. Provides 5-10x inference speedup compared to CPU.

**HailoRT Version:** 4.23.0 (January 2025 release)

**Prerequisites:**
- Load `landerpi-core` skill for connection and Docker setup
- Hailo-8 module installed on PCIe

## Architecture (ZeroMQ Bridge)

HailoRT 4.23 requires Python 3.13, but ROS2 Humble in Docker uses Python 3.10. The solution is a ZeroMQ bridge:

```
Host (Python 3.13 + HailoRT):
  └── hailo_inference_server.py → ZeroMQ server on tcp://localhost:5555
                                → runs YOLOv11 on Hailo-8 NPU
                                → managed by systemd (hailo-server.service)

Docker container (ROS2 Humble, Python 3.10):
  └── yolo_hailo_bridge node → subscribes /aurora/rgb/image_raw
                             → sends images to host via ZeroMQ
                             → receives detections
                             → publishes to /hazards, /yolo/detections
```

**Key Components:**
- `hailo8-int/host_server/hailo_inference_server.py` - ZeroMQ inference server (runs on host)
- `ros2_nodes/yolo_hailo_bridge/` - ROS2 node (runs in Docker container)
- `systemd/hailo-server.service` - Systemd service for inference server

## Key Features (HailoRT 4.23)

- **Python 3.13 support** - Works on Ubuntu 25.10
- **Trixie repository** - Uses `archive.raspberrypi.com/debian/ trixie main`
- **Simplified installation** - DKMS builds cleanly on modern kernels
- **Full Python API** - `python3-hailort` package available
- **ZeroMQ bridge** - Enables Hailo in Docker environments with different Python versions

## Quick Commands

| Command | Purpose |
|---------|---------|
| `uv run python deploy_hailo8.py check` | Check hardware/driver status |
| `uv run python deploy_hailo8.py install` | Install HailoRT 4.23 driver |
| `uv run python deploy_hailo8.py install --force` | Force reinstall |
| `uv run python deploy_hailo8.py deploy` | Upload models, server, and systemd service |
| `uv run python deploy_hailo8.py start` | Start inference server |
| `uv run python deploy_hailo8.py stop` | Stop inference server |
| `uv run python deploy_hailo8.py logs` | View server logs |
| `uv run python deploy_hailo8.py logs -f` | Follow server logs |
| `uv run python deploy_hailo8.py test` | Run validation test |
| `uv run python deploy_hailo8.py status` | Show device info |

## Setup Flow

```bash
# 1. Check hardware is detected
uv run python deploy_hailo8.py check

# 2. Install HailoRT 4.23 driver
uv run python deploy_hailo8.py install

# 3. Reboot if prompted
ssh user@robot 'sudo reboot'

# 4. Verify installation
uv run python deploy_hailo8.py check

# 5. Deploy models (after conversion)
uv run python deploy_hailo8.py deploy

# 6. Test with benchmark
uv run python deploy_hailo8.py test --benchmark
```

## Installation Details

The `install` command performs:
1. Adds Raspberry Pi **Trixie** APT repository with GPG key
2. Installs DKMS (required before Hailo packages)
3. Installs kernel headers matching running kernel
4. Removes old Hailo packages (if any)
5. Installs `hailort=4.23.0` (runtime and CLI)
6. Installs `hailort-pcie-driver` (kernel driver with DKMS)
7. Loads `hailo_pci` kernel module
8. Installs `python3-hailort` (Python API)

### Installed Packages

| Package | Version | Purpose |
|---------|---------|---------|
| `hailort` | 4.23.0 | Runtime library and CLI |
| `hailort-pcie-driver` | 4.23.0 | Kernel driver (DKMS) |
| `python3-hailort` | 4.23.0 | Python API bindings |

## Performance

| Backend | Model | FPS | Latency |
|---------|-------|-----|---------|
| CPU (Pi5) | YOLOv11n | 2-5 | 200-500ms |
| Hailo-8 | YOLOv11n | 25-40 | 25-40ms |

**Validated Performance (2026-01-03):**
- Hailo inference server: 21-25 FPS sustained during exploration
- ZeroMQ bridge latency: <5ms additional overhead
- Camera feed: 10-15 Hz (Aurora 930 on USB3)

## Firmware Version Note

M.2 Hailo-8 modules have firmware version 4.19.0 which is **not field-upgradeable**. This is normal - the 4.23 runtime works correctly with 4.19 firmware.

You will see this warning (it's cosmetic, inference works fine):
```
[HailoRT] [warning] Unsupported firmware operation. Host: 4.23.0, Device: 4.19.0
```

## Integration with Exploration

Use `--yolo-hailo` flag to enable Hailo-accelerated YOLO during exploration:

```bash
# Start exploration with Hailo YOLO (25-40 FPS)
uv run python deploy_explorer.py start --yolo-hailo --duration 10

# Combined with debug logging
uv run python deploy_explorer.py start --yolo-hailo --yolo-logging --duration 10

# Combined with ROS2 bag recording
uv run python deploy_explorer.py start --yolo-hailo --rosbag --duration 15
```

The Hailo bridge uses the same ROS2 interface as the CPU version:

```
/aurora/rgb/image_raw → [yolo_hailo_bridge] ←→ ZeroMQ ←→ [hailo_inference_server]
                                  ↓
                        /yolo/detections
                        /yolo/annotated_image (~14 Hz)
                        /hazards
```

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/yolo/detections` | vision_msgs/Detection2DArray | Bounding boxes and class info |
| `/yolo/annotated_image` | sensor_msgs/Image | Live camera feed with bboxes drawn |
| `/hazards` | std_msgs/String | JSON hazards for exploration controller |

### Live Annotated Video Stream

The `/yolo/annotated_image` topic publishes camera frames with YOLO bounding boxes at ~14 Hz:
- Red boxes: Hazard classes (person, dog, cat, etc.)
- Green boxes: Other detected objects
- Labels: Class name + confidence score

View with RViz by adding an Image display and setting topic to `/yolo/annotated_image`.

The `--yolo-hailo` flag automatically:
1. Starts the Hailo inference server (systemd service)
2. Restarts ROS2 stack with `USE_HAILO=true`
3. Builds and launches the `yolo_hailo_bridge` node
4. Bridge connects to inference server via ZeroMQ (tcp://localhost:5555)

## Pre-compiled Models (Recommended)

Download pre-compiled HEF models from Hailo Model Zoo - **no conversion needed**:

```bash
cd hailo8-int/conversion

# YOLOv11n (nano) - recommended for real-time on Pi
curl -L -o yolov11n.hef \
  "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.17.0/hailo8/yolov11n.hef"

# Copy to models and deploy
mkdir -p ../models && cp yolov11n.hef ../models/
uv run python deploy_hailo8.py deploy
```

### Available Models

| Model | mAP | Size | Use Case |
|-------|-----|------|----------|
| yolov11n | 39.0 | ~8 MB | Real-time on Pi (recommended) |
| yolov11s | 46.3 | ~18 MB | Balanced speed/accuracy |
| yolov11m | 51.1 | ~34 MB | Higher accuracy |
| yolov11l | 52.8 | ~50 MB | High accuracy |
| yolov11x | 54.1 | ~100 MB | Maximum accuracy |

**Full model list:** https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_object_detection.rst

## Manual Conversion (Advanced)

Only for custom-trained models. Requires x86_64 machine with Hailo SDK:

```bash
# 1. Export to ONNX with opset 11 (any machine)
yolo export model=yolo11n.pt format=onnx imgsz=640 opset=11

# 2. Convert to HEF (x86 Docker/Podman)
cd hailo8-int/conversion
docker build -t hailo-converter .
docker run -v $(pwd):/workspace hailo-converter python convert_yolo.py \
    --input yolo11n.onnx --output yolo11n_hailo.hef

# On ARM (Apple Silicon) with Podman:
podman build --platform linux/amd64 -t hailo-converter .
podman run --platform linux/amd64 -v $(pwd):/workspace hailo-converter python convert_yolo.py \
    --input yolo11n.onnx --output yolo11n_hailo.hef

# 3. Copy to models directory
cp yolo11n_hailo.hef ../models/

# 4. Deploy to robot
uv run python deploy_hailo8.py deploy
```

**Note:** Use `opset=11` - Hailo SDK doesn't support opset 22+.

## HailoRT 4.23 API Patterns

### Network Group Activation (Critical)

In HailoRT 4.23, network group activation **must be a context manager** that wraps `InferVStreams`:

```python
from hailo_platform import (
    HEF, VDevice, ConfigureParams, HailoStreamInterface,
    InferVStreams, InputVStreamParams, OutputVStreamParams
)

# Load model
hef = HEF(model_path)
vdevice = VDevice()
configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
network_group = vdevice.configure(hef, configure_params)[0]

# Create vstream params (HailoRT 4.23+ API)
input_params = InputVStreamParams.make(network_group)
output_params = OutputVStreamParams.make(network_group)
ng_params = network_group.create_params()

# CORRECT: Nested context managers
with network_group.activate(ng_params):  # Must wrap InferVStreams
    with InferVStreams(network_group, input_params, output_params) as pipeline:
        result = pipeline.infer({input_name: input_data})

# WRONG: These will fail with HAILO_NETWORK_GROUP_NOT_ACTIVATED
# network_group.activate()  # Not a context manager call
# with InferVStreams(...) as pipeline:  # Without activation wrapper
```

### Input Data Format

Hailo expects **uint8**, not float32:

```python
# CORRECT
input_data = np.expand_dims(image, axis=0).astype(np.uint8)

# WRONG - causes "dtype (float32) different than inferred dtype (uint8)" warning
input_data = image.astype(np.float32) / 255.0
```

### NMS Postprocess Output Format

Pre-compiled models from Hailo Model Zoo include NMS postprocessing. Output format is **list of class detections**, not raw tensor:

```python
# Output structure for yolov8_nms_postprocess:
# result['yolov11n/yolov8_nms_postprocess'] = [
#     [  # batch 0
#         np.array([...]),  # class 0 (person) detections: [[y1,x1,y2,x2,score], ...]
#         np.array([...]),  # class 1 (bicycle) detections
#         ...               # 80 COCO classes total
#     ]
# ]

output = result['yolov11n/yolov8_nms_postprocess']
batch_result = output[0]  # First batch

for class_id, class_detections in enumerate(batch_result):
    if len(class_detections) == 0:
        continue
    for det in class_detections:
        y1, x1, y2, x2, score = det[:5]  # Normalized 0-1 coordinates
        # Convert to pixel coordinates
        cx = (x1 + x2) / 2 * orig_width
        cy = (y1 + y2) / 2 * orig_height
```

### Persistent Inference Pipeline

For server applications, keep the pipeline active across requests:

```python
class HailoServer:
    def __init__(self, model_path):
        # ... setup code ...

        # Activate and keep open
        self.ng_params = self.network_group.create_params()
        self.activated_ng = self.network_group.activate(self.ng_params)
        self.activated_ng.__enter__()

        self.infer_pipeline = InferVStreams(
            self.network_group,
            self.input_vstream_params,
            self.output_vstream_params
        )
        self.infer_pipeline.__enter__()

    def infer(self, image):
        input_dict = {self.input_name: image}
        return self.infer_pipeline.infer(input_dict)

    def shutdown(self):
        self.infer_pipeline.__exit__(None, None, None)
        self.activated_ng.__exit__(None, None, None)
```

## Troubleshooting

### Device not found
```bash
# Check PCIe
lspci | grep -i hailo

# Check /boot/firmware/config.txt has:
dtparam=pciex1
```

### Driver not loaded
```bash
# Load manually
sudo modprobe hailo_pci

# Check DKMS
dkms status | grep hailo
```

### Permission denied
```bash
sudo usermod -aG hailo $USER
# Logout and login
```

### Python bindings not available
```bash
# Should work with 4.23 on Python 3.13
sudo apt-get install -y python3-hailort

# Verify
python3 -c "import hailo_platform; print(hailo_platform.__version__)"
```

### Upgrading from older version
```bash
# Force reinstall to update from 4.19/4.20 to 4.23
uv run python deploy_hailo8.py install --force
```

### HAILO_NETWORK_GROUP_NOT_ACTIVATED (Error 69)

**Symptom:**
```
[HailoRT] [error] CHECK failed - Trying to write to vstream before its network group is activated
[HailoRT] [error] CHECK_SUCCESS failed with status=HAILO_NETWORK_GROUP_NOT_ACTIVATED(69)
```

**Cause:** Network group activation must wrap InferVStreams as a context manager.

**Fix:**
```python
# WRONG
with InferVStreams(ng, input_params, output_params) as pipeline:
    pipeline.infer(...)

# CORRECT
with ng.activate(ng_params):
    with InferVStreams(ng, input_params, output_params) as pipeline:
        pipeline.infer(...)
```

### InferVStreams missing argument

**Symptom:**
```
InferVStreams.__init__() missing 1 required positional argument: 'output_vstreams_params'
```

**Cause:** HailoRT 4.23 changed the InferVStreams API.

**Fix:** Use separate input/output vstream params:
```python
# OLD API (pre-4.23)
with InferVStreams(ng, ng.create_params()) as pipeline:

# NEW API (4.23+)
input_params = InputVStreamParams.make(ng)
output_params = OutputVStreamParams.make(ng)
with InferVStreams(ng, input_params, output_params) as pipeline:
```

### 'list' object has no attribute 'shape'

**Symptom:** Postprocessing fails when accessing output tensor shape.

**Cause:** Hailo Model Zoo models include NMS postprocessing, outputting a list of class detections instead of raw tensor.

**Fix:** Handle list output format:
```python
output = result['yolov11n/yolov8_nms_postprocess']
if isinstance(output, list):
    # NMS format: list of 80 classes, each with [[y1,x1,y2,x2,score], ...]
    for class_id, detections in enumerate(output[0]):
        for det in detections:
            y1, x1, y2, x2, score = det[:5]
else:
    # Raw tensor format (custom models without NMS)
    ...
```

## Files

| File | Location | Purpose |
|------|----------|---------|
| `deploy_hailo8.py` | Project root | Deployment script (check/install/deploy/start/stop/logs) |
| `hailo_inference_server.py` | `hailo8-int/host_server/` | ZeroMQ inference server (runs on host) |
| `bridge_node.py` | `ros2_nodes/yolo_hailo_bridge/` | ROS2 ZeroMQ bridge node (runs in Docker) |
| `hailo-server.service` | `systemd/` | Systemd service for inference server |
| `INSTALLATION.md` | `hailo8-int/docs/` | Setup guide |
| `MODEL_CONVERSION.md` | `hailo8-int/docs/` | Conversion guide |
| `TROUBLESHOOTING.md` | `hailo8-int/docs/` | Troubleshooting guide |
| `*.hef` | `hailo8-int/models/` | Compiled models

## Optional: hailo-apps-infra

For additional ML tools:
```bash
cd ~
git clone --depth 1 https://github.com/hailo-ai/hailo-apps-infra.git
sudo apt-get install -y python3-venv pkg-config
cd hailo-apps-infra
sudo ./install.sh
```

Note: Full TAPPAS core has Raspberry Pi OS specific dependencies that don't work on Ubuntu.
