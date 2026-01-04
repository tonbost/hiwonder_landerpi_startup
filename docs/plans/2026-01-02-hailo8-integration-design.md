# Hailo 8 AI Accelerator Integration Design

**Date:** 2026-01-02
**Status:** Approved
**Author:** Claude + User

## Overview

Integrate the Hailo-8 AI accelerator (detected on PCIe at `0001:01:00.0`) to accelerate YOLOv11 object detection for the LanderPi robot's autonomous exploration system.

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Use case | YOLO acceleration + expandable | Immediate value, future-proof architecture |
| Runtime location | Native on host | Clean device access, kernel driver requirement |
| Model format | YOLOv11 → HEF (workstation conversion) | Pi5 RAM limitations, one-time conversion |
| Integration | Separate `yolo_hailo_node.py` | Clean separation, A/B testing, no risk to existing code |
| Deployment | `deploy_hailo8.py` at project root | Consistent with existing deploy_*.py pattern |

## Directory Structure

```
hiwonderSetup/
├── deploy_hailo8.py                 # Deployment script (at root)
├── hailo8-int/
│   ├── README.md                    # Setup guide and usage documentation
│   ├── docs/
│   │   ├── INSTALLATION.md          # Step-by-step HailoRT installation
│   │   ├── MODEL_CONVERSION.md      # PT → ONNX → HEF workflow
│   │   └── TROUBLESHOOTING.md       # Common issues and fixes
│   ├── models/
│   │   ├── yolo11n_hailo.hef        # Pre-converted YOLOv11n for Hailo-8
│   │   └── README.md                # Model sources and conversion notes
│   ├── drivers/
│   │   └── install_hailort.sh       # Driver installation commands
│   ├── ros2_nodes/
│   │   └── yolo_hailo/
│   │       ├── package.xml
│   │       ├── setup.py
│   │       └── yolo_hailo/
│   │           ├── __init__.py
│   │           └── yolo_hailo_node.py
│   ├── conversion/
│   │   ├── convert_yolo.py          # Runs on workstation
│   │   ├── Dockerfile
│   │   └── requirements.txt
│   └── validation/
│       └── test_hailo.py            # Validation tests
```

## Deployment Script Commands

`deploy_hailo8.py` follows existing Typer + Fabric pattern:

```bash
uv run python deploy_hailo8.py check      # Verify Hailo hardware + driver status
uv run python deploy_hailo8.py install    # Install HailoRT on robot
uv run python deploy_hailo8.py deploy     # Upload models + ROS2 node
uv run python deploy_hailo8.py test       # Run validation test
uv run python deploy_hailo8.py status     # Show Hailo device info
```

### Command Details

**check** - Read-only verification:
- `lspci | grep -i hailo` (hardware present?)
- `ls /dev/hailo*` (device node exists?)
- `hailortcli scan` (driver working?)
- `hailortcli fw-control identify` (firmware version)

**install** - Install HailoRT driver stack:
1. Add Hailo APT repository
2. `apt install hailort hailort-dkms`
3. Load kernel module (`modprobe hailo_pci`)
4. Verify `/dev/hailo0` exists
5. Install Python bindings (`pip install hailort`)
- Uses marker files (`~/.landerpi_setup/hailo_*`) for idempotency

**deploy** - Upload artifacts:
- Upload `hailo8-int/models/*.hef` to `~/landerpi/hailo/models/`
- Upload `hailo8-int/ros2_nodes/yolo_hailo/` to `~/landerpi/ros2_nodes/`

**test** - Validation:
- Run `hailortcli benchmark` on uploaded model
- Test inference on sample image
- Report FPS and latency

**status** - Device information:
- `hailortcli fw-control identify`
- Temperature, power, utilization

## ROS2 Node Architecture

`yolo_hailo_node.py` mirrors existing `yolo_node.py` interface:

```python
# Same ROS2 interface:
# Subscribes: /aurora/rgb/image_raw (sensor_msgs/Image)
# Publishes:  /yolo/detections (vision_msgs/Detection2DArray)
# Publishes:  /hazards (std_msgs/String, JSON)

class YoloHailoNode(Node):
    def __init__(self):
        # Parameters
        self.declare_parameter('hef_path', '~/landerpi/hailo/models/yolo11n_hailo.hef')
        self.declare_parameter('confidence_threshold', 0.4)

        # Initialize Hailo device
        from hailo_platform import HEF, VDevice, ConfigureParams
        self.vdevice = VDevice()  # Opens /dev/hailo0
        self.hef = HEF(hef_path)
        self.network_group = self._configure_network()

    def image_callback(self, msg):
        # Preprocess: RGB → BGR, resize to model input (640x640)
        # Run inference via Hailo async API
        # Postprocess: NMS, threshold filtering
        # Publish detections (same format as CPU node)
```

### Launch Configuration

Backend selection via launch parameter:

```python
use_hailo = LaunchConfiguration('use_hailo', default='false')

yolo_node = Node(
    package='yolo_hailo' if use_hailo else 'yolo_detector',
    executable='yolo_hailo_node' if use_hailo else 'yolo_node',
    ...
)
```

## Model Conversion Workflow

Conversion happens on workstation (Mac/PC), not on Pi:

```bash
# Step 1: Export PyTorch to ONNX
yolo export model=yolo11n.pt format=onnx imgsz=640

# Step 2: Run Hailo Dataflow Compiler (Docker on x86)
cd hailo8-int/conversion
docker build -t hailo-compiler .
docker run -v $(pwd):/workspace hailo-compiler python convert_yolo.py \
    --input yolo11n.onnx \
    --output ../models/yolo11n_hailo.hef \
    --calibration-images ./calibration_images/

# Step 3: Deploy HEF to robot
uv run python deploy_hailo8.py deploy
```

## Integration with Exploration

No changes needed to exploration code - same topic interface:

```
/aurora/rgb/image_raw → [yolo_hailo_node] → /hazards (JSON)
                                          → /yolo/detections
                                                    ↓
                            explorer.py ← sensor_fusion.py
```

Explorer usage remains identical:
```bash
uv run python deploy_explorer.py start --yolo --duration 10
```

## Expected Performance

| Metric | CPU (current) | Hailo-8 (expected) |
|--------|---------------|-------------------|
| Detection FPS | 2-5 | 25-40 |
| Latency | 200-500ms | 25-40ms |
| CPU freed | 100% core | ~5% (preprocessing) |
| Reaction distance @ 0.3m/s | 6-15cm | 0.75-1.2cm |

## Implementation Phases

### Phase 1: Foundation (Driver Setup)
- [ ] Create `hailo8-int/` directory structure
- [ ] Write `deploy_hailo8.py` with `check`, `install`, `status` commands
- [ ] Document installation steps in `INSTALLATION.md`
- [ ] Validate driver works: `/dev/hailo0` accessible, `hailortcli scan` succeeds

### Phase 2: Model Conversion
- [ ] Set up conversion Docker environment
- [ ] Convert YOLOv11n to HEF format
- [ ] Store in `hailo8-int/models/`
- [ ] Document process in `MODEL_CONVERSION.md`

### Phase 3: ROS2 Node
- [ ] Create `yolo_hailo_node.py` with same interface as existing node
- [ ] Add `deploy` command to `deploy_hailo8.py`
- [ ] Update launch config for backend selection
- [ ] Test with `/aurora/rgb/image_raw` feed

### Phase 4: Validation & Integration
- [ ] Add `test` command with benchmarks
- [ ] Verify `/hazards` topic works with explorer
- [ ] Performance comparison: CPU vs Hailo

### Phase 5: Documentation & Skills
- [ ] Update `README.md` with Hailo 8 section
- [ ] Update `CLAUDE.md` with Hailo commands
- [ ] Create new skill: `.claude/skills/landerpi-hailo/SKILL.md`
- [ ] Update `landerpi-yolo` skill - reference Hailo backend option
- [ ] Update `landerpi-dev` skill - add Hailo development patterns
- [ ] Update `landerpi-core` skill - add Hailo to system health checks

## State Tracking

Uses `~/.landerpi_setup/` markers (consistent with existing pattern):
- `hailo_repo_added`
- `hailo_driver_installed`
- `hailo_python_installed`
- `hailo_models_deployed`
