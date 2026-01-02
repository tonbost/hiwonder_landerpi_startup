# YOLOv11 to HEF Model Conversion

This guide covers converting YOLOv11 models to Hailo Executable Format (HEF) for the Hailo-8 accelerator.

## Prerequisites

- x86_64 Linux machine or Docker
- Hailo Dataflow Compiler (requires Hailo developer account)
- Python 3.8+
- YOLOv11 model (.pt file)

## Conversion Pipeline

```
YOLOv11 (.pt) → ONNX (.onnx) → HAR (.har) → HEF (.hef)
```

## Step 1: Export to ONNX

On any machine with ultralytics installed:

```bash
pip install ultralytics
yolo export model=yolo11n.pt format=onnx imgsz=640 opset=11
```

This creates `yolo11n.onnx`.

## Step 2: Convert to HEF (Docker Method)

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

## Step 3: Copy HEF to Models Directory

```bash
cp yolo11n_hailo.hef ../models/
```

## Step 4: Deploy to Robot

```bash
uv run python deploy_hailo8.py deploy
```

## Hailo Model Zoo

Pre-converted models are available in the Hailo Model Zoo:
- https://github.com/hailo-ai/hailo_model_zoo

Note: YOLOv11 may require manual conversion as it's newer than available zoo models.

## Calibration Images

For INT8 quantization, provide ~100 representative images:

```bash
mkdir -p calibration_images
# Copy sample images from your dataset
```

## Alternative: Hailo Model Zoo YOLOv8

If YOLOv11 conversion is problematic, YOLOv8 from Hailo Model Zoo can be used:

```bash
# From Hailo Model Zoo
git clone https://github.com/hailo-ai/hailo_model_zoo
cd hailo_model_zoo
pip install -e .

# Download pre-compiled HEF
hailo_model_zoo compile yolov8n
```

## Troubleshooting

### ONNX export fails
Ensure ultralytics version supports YOLOv11:
```bash
pip install --upgrade ultralytics
```

### HEF compilation OOM
Reduce batch size or use machine with more RAM (16GB+ recommended).

### Model accuracy degradation
Increase calibration images or try different quantization settings.

### Node name errors
Check ONNX model structure:
```bash
python -c "import onnx; m = onnx.load('yolo11n.onnx'); print([n.name for n in m.graph.input])"
```

## Expected Output Format

The HEF model should produce YOLO detection output:
- Bounding boxes (x, y, width, height)
- Class probabilities (80 COCO classes)
- Confidence scores

The `yolo_hailo_node.py` handles postprocessing of these outputs.
