"""Replaceable detector backend contract and OpenCV-DNN YOLOX implementation."""

from abc import ABC, abstractmethod
from pathlib import Path

import cv2
import numpy as np

from robot_perception.detection_utils import BackendDetection


COCO_CLASSES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
)


class DetectorBackend(ABC):
    @abstractmethod
    def infer(self, bgr_image: np.ndarray) -> list[BackendDetection]:
        """Return source-image pixel boxes without ROS-specific types."""


class OpenCvYoloXBackend(DetectorBackend):
    """YOLOX-S ONNX inference using the system OpenCV DNN runtime."""

    def __init__(
        self, model_path: str, input_size: int, device: str,
        confidence_threshold: float, nms_threshold: float,
    ):
        path = Path(model_path)
        if not path.is_file() or path.stat().st_size < 1024 * 1024:
            raise RuntimeError(f"YOLOX ONNX model is missing or invalid: {path}")
        if input_size <= 0 or input_size % 32:
            raise ValueError("input_size must be a positive multiple of 32")
        if device not in ("auto", "cpu", "cuda"):
            raise ValueError("device must be auto, cpu, or cuda")
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        if not 0.0 < nms_threshold <= 1.0:
            raise ValueError("nms_threshold must be in (0, 1]")

        self._input_size = input_size
        self._nms_threshold = nms_threshold
        self._confidence_threshold = confidence_threshold
        self._net = cv2.dnn.readNet(str(path))
        cuda_available = hasattr(cv2, "cuda") and cv2.cuda.getCudaEnabledDeviceCount() > 0
        self.device = "cuda" if device == "cuda" or (device == "auto" and cuda_available) else "cpu"
        if self.device == "cuda":
            if not cuda_available:
                raise RuntimeError("CUDA was requested but this OpenCV build has no CUDA device")
            self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        else:
            self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self._grids, self._strides = self._make_anchors(input_size)

    @staticmethod
    def _make_anchors(input_size):
        grids = []
        strides = []
        for stride in (8, 16, 32):
            count = input_size // stride
            xv, yv = np.meshgrid(np.arange(count), np.arange(count))
            grid = np.stack((xv, yv), axis=2).reshape(-1, 2)
            grids.append(grid)
            strides.append(np.full((grid.shape[0], 1), stride))
        return np.concatenate(grids).astype(np.float32), np.concatenate(strides).astype(np.float32)

    def infer(self, bgr_image):
        source_height, source_width = bgr_image.shape[:2]
        scale = min(self._input_size / source_height, self._input_size / source_width)
        resized = cv2.resize(
            cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB),
            (int(source_width * scale), int(source_height * scale)),
            interpolation=cv2.INTER_LINEAR,
        ).astype(np.float32)
        padded = np.full((self._input_size, self._input_size, 3), 114.0, dtype=np.float32)
        padded[: resized.shape[0], : resized.shape[1]] = resized
        blob = np.transpose(padded, (2, 0, 1))[np.newaxis]
        self._net.setInput(blob)
        raw = self._net.forward(self._net.getUnconnectedOutLayersNames())[0][0]

        boxes = raw[:, :4].copy()
        boxes[:, :2] = (boxes[:, :2] + self._grids) * self._strides
        boxes[:, 2:4] = np.exp(boxes[:, 2:4]) * self._strides
        boxes[:, 0:2] -= boxes[:, 2:4] / 2.0
        scores = raw[:, 4:5] * raw[:, 5:]
        class_ids = np.argmax(scores, axis=1)
        confidences = np.max(scores, axis=1)

        output = []
        for class_id in np.unique(class_ids):
            indices = np.where(class_ids == class_id)[0]
            class_boxes = boxes[indices]
            class_scores = confidences[indices]
            keep = cv2.dnn.NMSBoxes(
                class_boxes.tolist(), class_scores.tolist(),
                self._confidence_threshold, self._nms_threshold,
            )
            for local_index in np.asarray(keep).reshape(-1):
                index = indices[int(local_index)]
                x, y, width, height = boxes[index] / scale
                output.append(
                    BackendDetection(
                        class_id=int(class_id),
                        class_name=COCO_CLASSES[int(class_id)],
                        confidence=float(confidences[index]),
                        x=float(x), y=float(y), width=float(width), height=float(height),
                    )
                )
        return output
