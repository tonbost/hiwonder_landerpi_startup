---
name: landerpi-hailo
description: Hailo 8 AI accelerator integration for HiWonder LanderPi robot. Provides hardware-accelerated YOLOv11 inference at 25-40 FPS. Requires landerpi-core skill for connection setup. (project)
---

# LanderPi Hailo 8 AI Accelerator

## Overview

Hardware acceleration for YOLOv11 object detection using Hailo-8 NPU on Raspberry Pi 5. Provides 5-10x inference speedup compared to CPU.

**Prerequisites:**
- Load `landerpi-core` skill for connection and Docker setup
- Hailo-8 module installed on PCIe

## Quick Commands

| Command | Purpose |
|---------|---------|
| `uv run python deploy_hailo8.py check` | Check hardware/driver status |
| `uv run python deploy_hailo8.py install` | Install HailoRT driver |
| `uv run python deploy_hailo8.py deploy` | Upload models and ROS2 node |
| `uv run python deploy_hailo8.py test` | Run validation test |
| `uv run python deploy_hailo8.py status` | Show device info |

## Setup Flow

```bash
# 1. Check hardware is detected
uv run python deploy_hailo8.py check

# 2. Install driver (one-time)
uv run python deploy_hailo8.py install

# 3. Reboot if prompted
ssh user@robot 'sudo reboot'

# 4. Deploy models (after conversion)
uv run python deploy_hailo8.py deploy

# 5. Verify
uv run python deploy_hailo8.py test --benchmark
```

## Performance

| Backend | Model | FPS | Latency |
|---------|-------|-----|---------|
| CPU (Pi5) | YOLOv11n | 2-5 | 200-500ms |
| Hailo-8 | YOLOv11n | 25-40 | 25-40ms |

## Integration with Exploration

The Hailo YOLO node uses the same ROS2 interface as the CPU version:

```
/aurora/rgb/image_raw → [yolo_hailo_node] → /yolo/detections
                                          → /hazards
```

Enable via launch parameter:
```bash
# In docker-compose or launch file
use_hailo:=true
```

## Model Conversion

YOLOv11 models must be converted to HEF format:

```bash
# 1. Export to ONNX (any machine)
yolo export model=yolo11n.pt format=onnx imgsz=640

# 2. Convert to HEF (x86 machine with Hailo SDK)
cd hailo8-int/conversion
docker run -v $(pwd):/workspace hailo-converter python convert_yolo.py \
    --input yolo11n.onnx --output yolo11n_hailo.hef

# 3. Copy to models directory
cp yolo11n_hailo.hef ../models/

# 4. Deploy to robot
uv run python deploy_hailo8.py deploy
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
dkms status
```

### Permission denied
```bash
sudo usermod -aG hailo $USER
# Logout and login
```

## Files

| File | Location | Purpose |
|------|----------|---------|
| `deploy_hailo8.py` | Project root | Deployment script |
| `INSTALLATION.md` | `hailo8-int/docs/` | Setup guide |
| `MODEL_CONVERSION.md` | `hailo8-int/docs/` | Conversion guide |
| `TROUBLESHOOTING.md` | `hailo8-int/docs/` | Troubleshooting guide |
| `yolo_hailo_node.py` | `hailo8-int/ros2_nodes/yolo_hailo/` | ROS2 node |
| `*.hef` | `hailo8-int/models/` | Compiled models |
