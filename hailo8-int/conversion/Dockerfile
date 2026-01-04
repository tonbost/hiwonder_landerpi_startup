# Hailo Dataflow Compiler environment
FROM python:3.10-slim

WORKDIR /workspace

# Install system dependencies (gcc and graphviz-dev needed for pygraphviz compilation)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglib2.0-0 \
    graphviz \
    graphviz-dev \
    gcc \
    g++ \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (numpy<2 required - Hailo SDK compiled against NumPy 1.x)
RUN pip install --no-cache-dir "numpy<2" opencv-python-headless onnx

# Install Hailo Dataflow Compiler
COPY hailo_dataflow_compiler-3.33.0-py3-none-linux_x86_64.whl /tmp/
RUN pip install --no-cache-dir /tmp/hailo_dataflow_compiler-3.33.0-py3-none-linux_x86_64.whl \
    && rm /tmp/*.whl

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY convert_yolo.py .

CMD ["python", "convert_yolo.py", "--help"]
