"""cogproj_detector_base package."""

from .base_detector import BaseOreDetector, BoundingBoxValidationError, DetectionResult
from .mock_detector import MockDetector, OreDetectorPlugin
from .plugin_loader import (
    PluginConfigError,
    PluginError,
    PluginInterfaceError,
    PluginNotFoundError,
    load_detector_plugin,
)

__all__ = [
    "BaseOreDetector",
    "DetectionResult",
    "BoundingBoxValidationError",
    "MockDetector",
    "OreDetectorPlugin",
    "PluginError",
    "PluginNotFoundError",
    "PluginInterfaceError",
    "PluginConfigError",
    "load_detector_plugin",
]

