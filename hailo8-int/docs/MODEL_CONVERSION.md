# YOLOv11 Models for Hailo-8

This guide covers getting YOLOv11 models in Hailo Executable Format (HEF) for the Hailo-8 accelerator.

## Pre-compiled Models (Recommended)

Hailo provides pre-compiled HEF models that work out of the box - **no conversion needed**.

### Available YOLOv11 Models

| Model | mAP | Size | Use Case |
|-------|-----|------|----------|
| yolov11n | 39.0 | ~8 MB | Real-time on Pi (recommended) |
| yolov11s | 46.3 | ~18 MB | Balanced speed/accuracy |
| yolov11m | 51.1 | ~34 MB | Higher accuracy |
| yolov11l | 52.8 | ~50 MB | High accuracy |
| yolov11x | 54.1 | ~100 MB | Maximum accuracy |

### Download Pre-compiled HEF

```bash
cd hailo8-int/conversion

# YOLOv11n (nano) - recommended for Pi real-time inference
curl -L -o yolov11n.hef \
  "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.17.0/hailo8/yolov11n.hef"

# YOLOv11m (medium) - higher accuracy
curl -L -o yolov11m.hef \
  "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.17.0/hailo8/yolov11m.hef"

# Copy to models directory
mkdir -p ../models
cp yolov11n.hef ../models/
```

### Deploy to Robot

```bash
uv run python deploy_hailo8.py deploy
```

## Hailo Model Zoo Reference

Full list of pre-compiled models for Hailo-8:
- **Model List**: https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_object_detection.rst
- **Repository**: https://github.com/hailo-ai/hailo_model_zoo

Other available YOLO variants:
- **YOLOv10**: yolov10n, yolov10s, yolov10b, yolov10x (mAP 38.5-53.7)
- **YOLOv8**: yolov8n through yolov8x

---

## Manual Conversion (Advanced)

Only needed if pre-compiled models don't meet your requirements (e.g., custom trained models).

### Prerequisites

- x86_64 Linux machine or Docker/Podman
- Hailo Dataflow Compiler (requires Hailo developer account)
- Python 3.8+
- YOLOv11 model (.pt file)

### Conversion Pipeline

```
YOLOv11 (.pt) → ONNX (.onnx) → HAR (.har) → HEF (.hef)
```

### Step 1: Export to ONNX

On any machine with ultralytics installed:

```bash
pip install ultralytics
yolo export model=yolo11n.pt format=onnx imgsz=640 opset=11
```

**Important**: Use `opset=11` - the Hailo SDK doesn't support opset 22+.

This creates `yolo11n.onnx`.

### Step 2: Convert to HEF (Docker Method)

The Hailo Dataflow Compiler only runs on x86_64. Use Docker:

```bash
cd hailo8-int/conversion

# Build converter image
docker build -t hailo-converter .

# Run conversion
docker run -v $(pwd):/workspace hailo-converter \
    python convert_yolo.py \
    --input /workspace/yolo11n.onnx \
    --output /workspace/yolo11n_hailo.hef
```

#### ARM Machines (Apple Silicon, etc.)

On ARM machines, you must specify the x86_64 platform explicitly. With Podman:

```bash
cd hailo8-int/conversion

# Build with platform emulation (required on ARM)
podman build --platform linux/amd64 -t hailo-converter .

# Run conversion
podman run --platform linux/amd64 -v $(pwd):/workspace hailo-converter \
    python convert_yolo.py \
    --input /workspace/yolo11n.onnx \
    --output /workspace/yolo11n_hailo.hef
```

With Docker on ARM:

```bash
docker build --platform linux/amd64 -t hailo-converter .
docker run --platform linux/amd64 -v $(pwd):/workspace hailo-converter \
    python convert_yolo.py \
    --input /workspace/yolo11n.onnx \
    --output /workspace/yolo11n_hailo.hef
```

Note: x86_64 emulation on ARM is slower but works correctly.

### Step 3: Copy HEF to Models Directory

```bash
cp yolo11n_hailo.hef ../models/
```

### Step 4: Deploy to Robot

```bash
uv run python deploy_hailo8.py deploy
```

### Calibration Images

For INT8 quantization, provide ~100 representative images:

```bash
mkdir -p calibration_images
# Copy sample images from your dataset
```

### Using Hailo Model Zoo CLI

Alternative method using Hailo's tools directly:

```bash
# Clone Hailo Model Zoo
git clone https://github.com/hailo-ai/hailo_model_zoo
cd hailo_model_zoo
pip install -e .

# Compile a model (downloads and converts)
hailo_model_zoo compile yolov8n
```

## Troubleshooting

### ONNX export fails
Ensure ultralytics version supports YOLOv11:
```bash
pip install --upgrade ultralytics
```

### ONNX opset not supported
Error: `ONNX Runtime only guarantees support for models stamped with official released onnx opset versions`

Solution: Re-export with opset 11:
```bash
yolo export model=yolo11n.pt format=onnx imgsz=640 opset=11
```

### UnsupportedShuffleLayerError / DFL layer
Error: `UnsupportedShuffleLayerError in op /model.23/dfl/Reshape`

This is a known issue with YOLOv11's DFL (Distribution Focal Loss) layer. The conversion script uses `end_node_names=["/model.23/Concat_3"]` to exclude this layer. DFL postprocessing is handled on CPU.

**Recommendation**: Use pre-compiled models from Hailo Model Zoo instead.

### HEF compilation OOM
Reduce batch size or use machine with more RAM (16GB+ recommended).

### Model accuracy degradation
Increase calibration images or try different quantization settings.

### NumPy version conflict
Error: `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`

Solution: The Dockerfile pins `numpy<2`. Rebuild the container if needed.

### Node name errors
Check ONNX model structure:
```bash
python -c "import onnx; m = onnx.load('yolo11n.onnx'); print([n.name for n in m.graph.input])"
```

## Expected Output Format

The HEF model produces YOLO detection output:
- Bounding boxes (x, y, width, height)
- Class probabilities (80 COCO classes)
- Confidence scores

The `yolo_hailo_node.py` handles postprocessing of these outputs.
