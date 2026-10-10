# CogProj — Detector Plugin & Model Integration Guide

This guide describes how to integrate a trained ore detection model into CogProj.
The detector interface is **framework-independent**, allowing any machine learning
framework (ONNX Runtime, TFLite, PyTorch, TensorRT, etc.) to be plugged in without
modifying the core ROS 2 nodes.

---

## 1. Architecture Overview

CogProj uses dynamic plugin loading to decouple model runtimes from the ROS 2 pipeline:

```
                      ┌──────────────────────────────────────┐
                      │            DetectionNode             │
                      │  (/cogproj/image_raw -> detections)  │
                      └──────────────────┬───────────────────┘
                                         │
                             calls BaseOreDetector API
                                         │
                      ┌──────────────────▼───────────────────┐
                      │          OreDetectorPlugin           │
                      │  (Subclasses BaseOreDetector ABC)    │
                      └──────────────────┬───────────────────┘
                                         │
                            Loads weights & runs inference
                                         │
                      ┌──────────────────▼───────────────────┐
                      │  Trained Model / Inference Runtime   │
                      │   (ONNX, TFLite, PyTorch, YOLO, etc.)│
                      └──────────────────────────────────────┘
```

The core components live in `cogproj_detector_base`:
* **`BaseOreDetector`**: Abstract Base Class defining the plugin contract.
* **`DetectionResult`**: Standardized dataclass for detected bounding boxes.
* **`load_detector_plugin()`**: Dynamic loader that imports and instantiates the plugin at runtime.

---

## 2. Model Package Structure

When preparing a model for CogProj, place the files in a self-contained directory:

```
/home/veerobot/CogProj/models/my_ore_model/
├── adapter.py              # REQUIRED: Plugin entry point exporting OreDetectorPlugin
├── config.yaml             # OPTIONAL: Model parameters, class labels, thresholds
└── weights/                # Model weight files (.pt, .onnx, .tflite, etc.)
    └── model_file.ext
```

> **Note:** Binary model weights are excluded from Git via `.gitignore` to prevent large binary files in the repository.

---

## 3. The Detector Plugin Contract

The file `adapter.py` must define a class named **`OreDetectorPlugin`** that inherits from **`BaseOreDetector`**.

### Required Methods

```python
from typing import Any, Dict, List, Optional
import numpy as np
from cogproj_detector_base import BaseOreDetector, DetectionResult


class OreDetectorPlugin(BaseOreDetector):
    """Adapter bridging a custom ore detection model into CogProj."""

    def load(self, model_dir: str, config: Optional[Dict[str, Any]] = None) -> bool:
        """Initialize runtime, load weights, and prepare for inference.

        Args:
            model_dir: Absolute path to the model directory.
            config: Optional dictionary loaded from config.yaml or ROS params.

        Returns:
            True if loading was successful, False otherwise.
        """
        ...

    def predict(self, frame_bgr: np.ndarray) -> List[DetectionResult]:
        """Execute object detection on a single camera frame.

        Args:
            frame_bgr: Camera image in BGR format as a NumPy uint8 array [H, W, 3].
                       Note: Input resolution is determined by the camera stream
                       (e.g., 320x240 or camera driver resolution).

        Returns:
            List of validated DetectionResult objects with coordinates mapped
            back to the original camera frame pixel coordinates.
        """
        ...

    def get_metadata(self) -> Dict[str, Any]:
        """Return information describing the model.

        Returns:
            Dictionary with keys such as 'model_name', 'version', 'classes', 'framework'.
        """
        ...

    def unload(self) -> None:
        """Release GPU/CPU memory, accelerators, and runtime resources."""
        ...
```

---

## 4. Coordinate System & Image Resolution Mapping

> [!IMPORTANT]
> **Resolution Independence:** The incoming `frame_bgr` passed to `predict()` has the
> native resolution of the camera stream (e.g., $320 \times 240$ or whatever resolution the
> camera publisher provides: `orig_h = frame_bgr.shape[0]`, `orig_w = frame_bgr.shape[1]`).

Different detection models expect different fixed input dimensions (e.g., $640 \times 640$,
$416 \times 416$, $300 \times 300$, $224 \times 224$):

1. **Preprocessing:** The adapter must preprocess (resize, letterbox, normalize) the incoming
   camera frame to the input shape expected by the model.
2. **Inference:** The model produces raw bounding boxes (often normalized $[0, 1]$ or relative
   to the model's resized dimensions).
3. **Postprocessing (Coordinate Remapping):** The adapter **must map the predicted coordinates
   back to the original camera frame coordinate system** before instantiating `DetectionResult`:
   * $x_{\min}, x_{\max} \in [0, \text{orig\_w}]$
   * $y_{\min}, y_{\max} \in [0, \text{orig\_h}]$

Downstream tracking and visualization nodes operate in the original camera pixel space.
If coordinates are left normalized or scaled to model dimensions, tracking distance gates
and visual overlays will be misaligned.

---

## 5. `DetectionResult` Data Structure

Every detection returned by `predict()` must be an instance of `DetectionResult`:

```python
DetectionResult(
    class_id=1,                 # Integer class ID (>= 0)
    class_name="Hematite",       # Human-readable class label
    confidence=0.88,            # Float in range [0.0, 1.0]
    x_min=45.0,                 # Left coordinate in camera frame pixels
    y_min=30.0,                 # Top coordinate in camera frame pixels
    x_max=115.0,                # Right coordinate in camera frame pixels
    y_max=95.0,                 # Bottom coordinate in camera frame pixels
)
```

### Validation Constraints
`DetectionResult` enforces:
* `0.0 <= confidence <= 1.0` and finite (no NaN or Inf)
* Coordinates must be finite and non-negative ($x \ge 0$, $y \ge 0$)
* Geometrically valid bounds ($x_{\min} \le x_{\max}$ and $y_{\min} \le y_{\max}$)
* Automatically computes `center_x`, `center_y`, `width`, and `height`.

---

## 6. Illustrative Adapter Templates

> [!WARNING]
> **The code snippets below are illustrative conceptual templates, NOT drop-in adapters.**
> Output tensor shapes and postprocessing logic vary significantly depending on the specific
> detector head (YOLOv8, YOLOv5, SSD, Faster R-CNN, etc.). Adapt this logic to match your model's
> exact output specification.

### Illustrative Template A: ONNX Runtime Concept

```python
# ILLUSTRATIVE TEMPLATE ONLY - Adapt tensor indices and postprocessing to your model
import os
from typing import Any, Dict, List, Optional
import cv2
import numpy as np
import onnxruntime as ort
from cogproj_detector_base import BaseOreDetector, DetectionResult


class OreDetectorPlugin(BaseOreDetector):
    """Conceptual template demonstrating ONNX model loading and inference."""

    def __init__(self):
        self.session = None
        self.class_names = []
        self.input_size = (640, 640)  # Example model dimension

    def load(self, model_dir: str, config: Optional[Dict[str, Any]] = None) -> bool:
        config = config or {}
        self.class_names = config.get("classes", ["Ore_A", "Ore_B"])
        onnx_path = os.path.join(model_dir, "weights", "ore_detector.onnx")

        if not os.path.isfile(onnx_path):
            return False

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4
        self.session = ort.InferenceSession(onnx_path, opts, providers=["CPUExecutionProvider"])
        return True

    def predict(self, frame_bgr: np.ndarray) -> List[DetectionResult]:
        if self.session is None or frame_bgr is None:
            return []

        orig_h, orig_w = frame_bgr.shape[:2]

        # Preprocessing: resize to model input dimensions and normalize
        resized = cv2.resize(frame_bgr, self.input_size)
        blob = resized.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[np.newaxis, ...]

        input_name = self.session.get_inputs()[0].name
        raw_outputs = self.session.run(None, {input_name: blob})

        # Remapping: Scale predicted coordinates back to camera resolution [orig_w, orig_h]
        # (Replace this placeholder decoding with your model's specific tensor parsing)
        scale_x = orig_w / float(self.input_size[0])
        scale_y = orig_h / float(self.input_size[1])

        results = []
        # Example decoding loop (illustrative):
        # for det in decoded_detections:
        #     results.append(DetectionResult(
        #         class_id=det.class_id,
        #         class_name=self.class_names[det.class_id],
        #         confidence=det.confidence,
        #         x_min=det.x_min * scale_x,
        #         y_min=det.y_min * scale_y,
        #         x_max=det.x_max * scale_x,
        #         y_max=det.y_max * scale_y,
        #     ))
        return results

    def get_metadata(self) -> Dict[str, Any]:
        return {
            "model_name": "ONNX_Ore_Detector_Template",
            "framework": "onnxruntime",
            "classes": self.class_names,
        }

    def unload(self) -> None:
        self.session = None
```

### Illustrative Template B: TFLite Runtime Concept

```python
# ILLUSTRATIVE TEMPLATE ONLY - Adapt tensor indices and postprocessing to your model
import os
from typing import Any, Dict, List, Optional
import cv2
import numpy as np
from cogproj_detector_base import BaseOreDetector, DetectionResult

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    from tensorflow.lite.python import interpreter as tflite


class OreDetectorPlugin(BaseOreDetector):
    """Conceptual template demonstrating TFLite model loading and inference."""

    def __init__(self):
        self.interpreter = None
        self.class_names = []

    def load(self, model_dir: str, config: Optional[Dict[str, Any]] = None) -> bool:
        config = config or {}
        self.class_names = config.get("classes", ["Ore_Alpha", "Ore_Beta"])
        model_path = os.path.join(model_dir, "weights", "ore_detector.tflite")

        if not os.path.isfile(model_path):
            return False

        self.interpreter = tflite.Interpreter(model_path=model_path, num_threads=4)
        self.interpreter.allocate_tensors()
        return True

    def predict(self, frame_bgr: np.ndarray) -> List[DetectionResult]:
        if self.interpreter is None or frame_bgr is None:
            return []

        orig_h, orig_w = frame_bgr.shape[:2]
        # Preprocessing, inference, and scaling back to [orig_w, orig_h] ...
        return []

    def get_metadata(self) -> Dict[str, Any]:
        return {
            "model_name": "TFLite_Ore_Detector_Template",
            "framework": "tflite",
            "classes": self.class_names,
        }

    def unload(self) -> None:
        self.interpreter = None
```

---

## 7. Configuring CogProj to Use the Model

Edit `/home/veerobot/CogProj/src/cogproj_bringup/config/cogproj_config.yaml`:

```yaml
detection_node:
  ros__parameters:
    enabled: true                                                 # Enable detection
    test_mode: false                                              # Disable MockDetector
    model_directory: "/home/veerobot/CogProj/models/my_ore_model" # Directory containing adapter.py
    confidence_threshold: 0.60                                    # Desired cutoff
```

---

## 8. Testing the Adapter in Isolation

Before launching the full ROS 2 system, verify the adapter in a standalone Python script:

```python
import numpy as np
from cogproj_detector_base import load_detector_plugin

# 1. Load the plugin using the CogProj dynamic loader API
detector = load_detector_plugin(
    model_dir="/home/veerobot/CogProj/models/my_ore_model",
    adapter_filename="adapter.py",  # Defaults to adapter.py
    auto_init=True,
)

# 2. Check metadata
print("Metadata:", detector.get_metadata())

# 3. Test inference on a blank test image
test_frame = np.zeros((240, 320, 3), dtype=np.uint8)
detections = detector.predict(test_frame)
print(f"Detections returned: {len(detections)}")

# 4. Unload
detector.unload()
print("Plugin unloaded successfully.")
```

---

## 9. Troubleshooting

| Error | Cause | Resolution |
|---|---|---|
| `PluginNotFoundError` | Directory or `adapter.py` missing | Ensure `adapter.py` exists inside `model_directory` |
| `PluginInterfaceError` | `OreDetectorPlugin` class not found or doesn't inherit `BaseOreDetector` | Verify class name and inheritance in `adapter.py` |
| `PluginError: load() returned False` | Weight file missing or runtime failed to init | Verify file paths inside `load()` method |
| `BoundingBoxValidationError` | Box coordinates negative or confidence > 1.0 | Clamp coordinates and confidence before creating `DetectionResult` |
| `No detector loaded` | `model_directory` parameter is empty in ROS YAML | Set `model_directory` in `cogproj_config.yaml` |
