"""Dynamic plugin loader for CogProj ore detectors.

Safely discovers, validates, and loads user detector adapters that conform
to the BaseOreDetector contract.
"""

import importlib.util
import os
import sys
from typing import Any, Dict, Optional, Type

from .base_detector import BaseOreDetector

# Name of the class that must be exported by adapter.py
REQUIRED_PLUGIN_CLASS_NAME = "OreDetectorPlugin"
DEFAULT_ADAPTER_FILENAME = "adapter.py"
DEFAULT_CONFIG_FILENAME = "config.yaml"


class PluginError(Exception):
    """Base exception for all plugin loading errors."""
    pass


class PluginNotFoundError(PluginError):
    """Raised when the specified model directory or adapter file cannot be found."""
    pass


class PluginInterfaceError(PluginError):
    """Raised when the plugin does not implement BaseOreDetector or export OreDetectorPlugin."""
    pass


class PluginConfigError(PluginError):
    """Raised when the plugin configuration file is corrupted or unreadable."""
    pass


def load_plugin_config(config_path: str) -> Dict[str, Any]:
    """Safely load YAML configuration file if present."""
    if not os.path.isfile(config_path):
        return {}

    try:
        import yaml
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            return data if isinstance(data, dict) else {}
    except ImportError:
        # Fallback if PyYAML is unavailable: return empty dict or basic parse
        return {}
    except Exception as e:
        raise PluginConfigError(f"Failed to parse config file '{config_path}': {e}") from e


def load_detector_plugin(
    model_dir: str,
    adapter_filename: Optional[str] = None,
    config_path: Optional[str] = None,
    auto_init: bool = True,
) -> BaseOreDetector:
    """Dynamically load and instantiate an OreDetectorPlugin from a directory.

    Args:
        model_dir: Directory containing the model and adapter.py.
        adapter_filename: Optional custom filename for adapter (defaults to 'adapter.py').
        config_path: Optional explicit path to config.yaml.
        auto_init: If True, immediately calls detector.load(model_dir, config).

    Returns:
        An instantiated BaseOreDetector plugin.

    Raises:
        PluginNotFoundError: If directory or adapter file does not exist.
        PluginInterfaceError: If OreDetectorPlugin is missing or invalid.
        PluginError: If initialization fails.
    """
    if not os.path.isdir(model_dir):
        raise PluginNotFoundError(f"Model directory does not exist: '{model_dir}'")

    adapter_name = adapter_filename or DEFAULT_ADAPTER_FILENAME
    adapter_path = os.path.join(model_dir, adapter_name)

    if not os.path.isfile(adapter_path):
        raise PluginNotFoundError(
            f"Plugin adapter file '{adapter_name}' not found in directory: '{model_dir}'"
        )

    # Resolve configuration
    cfg_file = config_path or os.path.join(model_dir, DEFAULT_CONFIG_FILENAME)
    config = load_plugin_config(cfg_file) if os.path.isfile(cfg_file) else {}

    # Unique module name to prevent namespace collisions
    module_name = f"cogproj_plugin_{abs(hash(adapter_path))}"

    try:
        spec = importlib.util.spec_from_file_location(module_name, adapter_path)
        if spec is None or spec.loader is None:
            raise PluginInterfaceError(f"Unable to load import specification from '{adapter_path}'")

        module = importlib.util.module_from_spec(spec)
        # Add model_dir to sys.path temporarily to allow adapter local imports
        sys.path.insert(0, model_dir)
        try:
            spec.loader.exec_module(module)
        finally:
            if sys.path and sys.path[0] == model_dir:
                sys.path.pop(0)

    except Exception as e:
        raise PluginError(f"Failed to execute adapter module '{adapter_path}': {e}") from e

    # Verify class exists
    if not hasattr(module, REQUIRED_PLUGIN_CLASS_NAME):
        raise PluginInterfaceError(
            f"Adapter '{adapter_path}' does not define required class '{REQUIRED_PLUGIN_CLASS_NAME}'"
        )

    plugin_cls = getattr(module, REQUIRED_PLUGIN_CLASS_NAME)

    if not isinstance(plugin_cls, type) or not issubclass(plugin_cls, BaseOreDetector):
        raise PluginInterfaceError(
            f"Class '{REQUIRED_PLUGIN_CLASS_NAME}' in '{adapter_path}' must subclass 'BaseOreDetector'"
        )

    # Instantiate detector
    try:
        instance: BaseOreDetector = plugin_cls()
    except Exception as e:
        raise PluginError(f"Failed to instantiate '{REQUIRED_PLUGIN_CLASS_NAME}': {e}") from e

    if auto_init:
        success = instance.load(model_dir=model_dir, config=config)
        if not success:
            raise PluginError(
                f"Plugin '{REQUIRED_PLUGIN_CLASS_NAME}' load() method returned False for '{model_dir}'"
            )

    return instance

