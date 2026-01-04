
import pytest
try:
    from ultralytics import YOLO
except ImportError:
    pytest.skip("ultralytics not installed", allow_module_level=True)

def test_yolo_model_download_and_load():
    """Verify that YOLO model can be loaded (and downloaded if needed)."""
    try:
        model = YOLO('yolo11n.pt')
        assert model is not None, "Model is None"
    except Exception as e:
        pytest.fail(f"Failed to load model: {e}")
