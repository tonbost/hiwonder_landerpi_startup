# Hailo Dataflow Compiler environment
# Note: Requires Hailo developer account for DFC installation
FROM python:3.10-slim

WORKDIR /workspace

# Install dependencies
RUN pip install numpy opencv-python-headless onnx

# Note: hailo_sdk requires manual installation from Hailo
# Download from: https://hailo.ai/developer-zone/
# COPY hailo_sdk-*.whl /tmp/
# RUN pip install /tmp/hailo_sdk-*.whl

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY convert_yolo.py .

CMD ["python", "convert_yolo.py", "--help"]
