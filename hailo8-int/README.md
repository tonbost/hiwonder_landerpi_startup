# Hailo 8 AI Accelerator Integration

Hardware acceleration for YOLOv11 inference on LanderPi robot using Hailo-8 NPU.

## Quick Start

```bash
# Check Hailo hardware status
uv run python deploy_hailo8.py check

# Install HailoRT driver
uv run python deploy_hailo8.py install

# Deploy models and ROS2 node
uv run python deploy_hailo8.py deploy

# Run validation test
uv run python deploy_hailo8.py test

# Show device status
uv run python deploy_hailo8.py status
```

## Documentation

- [Installation Guide](docs/INSTALLATION.md)
- [Model Conversion](docs/MODEL_CONVERSION.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)

## Performance

| Backend | Model | FPS | Latency |
|---------|-------|-----|---------|
| CPU (Pi5) | YOLOv11n | 2-5 | 200-500ms |
| Hailo-8 | YOLOv11n | 25-40 | 25-40ms |
