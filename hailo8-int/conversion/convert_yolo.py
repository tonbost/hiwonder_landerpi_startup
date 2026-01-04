#!/usr/bin/env python3
"""
Convert YOLOv11 ONNX model to Hailo HEF format.

Requires Hailo Dataflow Compiler (hailo_sdk).

IMPORTANT: Export ONNX with opset 11 (Hailo SDK doesn't support opset 22):
    yolo export model=yolo11n.pt format=onnx imgsz=640 opset=11
"""
import argparse
from pathlib import Path

# YOLOv11 end nodes - exclude DFL layer which Hailo doesn't support
# The DFL postprocessing will be done on CPU
YOLO11_END_NODES = ["/model.23/Concat_3"]


def convert_onnx_to_hef(input_path: str, output_path: str, calibration_dir: str = None, end_nodes: list = None):
    """Convert ONNX model to HEF format."""
    try:
        from hailo_sdk_client import ClientRunner
    except ImportError:
        print("Error: hailo_sdk not installed.")
        print("Download from https://hailo.ai/developer-zone/")
        return False

    if end_nodes is None:
        end_nodes = YOLO11_END_NODES

    print(f"Converting {input_path} to {output_path}")
    print(f"End nodes: {end_nodes}")

    # Create runner
    runner = ClientRunner(hw_arch="hailo8")

    # Parse ONNX
    print("Parsing ONNX model...")
    hn, npz = runner.translate_onnx_model(
        input_path,
        "yolo11n",
        end_node_names=end_nodes,
    )

    # Optimize
    print("Optimizing model...")
    runner.optimize(hn, npz)

    # Quantize (if calibration images provided)
    if calibration_dir:
        print(f"Quantizing with calibration images from {calibration_dir}...")
        # Add calibration logic here

    # Compile
    print("Compiling to HEF...")
    hef = runner.compile()

    # Save
    with open(output_path, "wb") as f:
        f.write(hef)

    print(f"Saved HEF to {output_path}")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Convert YOLO ONNX to Hailo HEF",
        epilog="IMPORTANT: Export ONNX with opset 11: yolo export model=yolo11n.pt format=onnx imgsz=640 opset=11"
    )
    parser.add_argument("--input", required=True, help="Input ONNX file")
    parser.add_argument("--output", required=True, help="Output HEF file")
    parser.add_argument("--calibration", help="Calibration images directory")
    parser.add_argument("--end-nodes", nargs="+", default=None,
                        help=f"End node names (default: {YOLO11_END_NODES})")

    args = parser.parse_args()

    if not Path(args.input).exists():
        print(f"Error: Input file not found: {args.input}")
        return

    convert_onnx_to_hef(args.input, args.output, args.calibration, args.end_nodes)


if __name__ == "__main__":
    main()
