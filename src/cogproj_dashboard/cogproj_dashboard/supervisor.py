"""Control Supervisor for CogProj Operator Console.

Enforces strict safety boundaries:
1. Mutual exclusion between STOPPED, MANUAL, and AUTONOMOUS modes.
2. Default state is always STOPPED.
3. Velocity clamping (linear <= 0.35 m/s, angular <= 0.80 rad/s).
4. Deadman switch enforcement (must be active for motion).
5. Watchdog timeout: auto-stop if no command received for 0.3 seconds.
6. Immediate stop on key release, focus loss, tab hide, or connection drop.
"""

from enum import Enum
import threading
import time
from typing import Callable, Optional, Tuple


class ControlMode(str, Enum):
    STOPPED = "STOPPED"
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"


class ControlSupervisor:
    """ROS-side safety supervisor for vehicle motion commands."""

    def __init__(
        self,
        max_linear_x: float = 0.35,
        max_angular_z: float = 0.80,
        command_timeout_sec: float = 0.30,
        on_velocity_output: Optional[Callable[[float, float, bool], None]] = None,
        on_mode_changed: Optional[Callable[[ControlMode, ControlMode], None]] = None,
        on_stop_requested: Optional[Callable[[], None]] = None,
    ) -> None:
        self.max_linear_x = abs(float(max_linear_x))
        self.max_angular_z = abs(float(max_angular_z))
        self.command_timeout_sec = max(0.05, float(command_timeout_sec))

        self._on_velocity_output = on_velocity_output
        self._on_mode_changed = on_mode_changed
        self._on_stop_requested = on_stop_requested

        self._lock = threading.Lock()
        self._mode = ControlMode.STOPPED
        self._deadman_active = False
        self._last_command_time = 0.0
        self._current_linear_x = 0.0
        self._current_angular_z = 0.0

    def _emit_velocity(self, linear_x: float, angular_z: float, deadman: bool) -> None:
        """Call on_velocity_output supporting both (lin, ang, deadman) and (lin, ang) signatures."""
        if not self._on_velocity_output:
            return
        try:
            self._on_velocity_output(linear_x, angular_z, deadman)
        except TypeError:
            self._on_velocity_output(linear_x, angular_z)

    @property
    def mode(self) -> ControlMode:
        with self._lock:
            return self._mode

    @property
    def is_moving(self) -> bool:
        with self._lock:
            return abs(self._current_linear_x) > 1e-4 or abs(self._current_angular_z) > 1e-4

    def set_mode(self, new_mode_str: str) -> Tuple[bool, str]:
        """Transition control mode with safe state reset."""
        try:
            target_mode = ControlMode(new_mode_str.upper())
        except ValueError:
            return False, f"Invalid mode: {new_mode_str}. Must be STOPPED, MANUAL, or AUTONOMOUS."

        with self._lock:
            old_mode = self._mode
            if old_mode == target_mode:
                return True, f"Mode already {target_mode.value}"

            # Always stop motion on mode switch
            self._zero_motion_locked()
            self._mode = target_mode

        if self._on_mode_changed:
            self._on_mode_changed(old_mode, target_mode)

        return True, f"Mode switched from {old_mode.value} to {target_mode.value}"

    def receive_teleop_command(
        self, linear_x: float, angular_z: float, deadman: bool
    ) -> Tuple[bool, str]:
        """Validate, clamp, and process manual keyboard commands."""
        with self._lock:
            if self._mode != ControlMode.MANUAL:
                self._zero_motion_locked()
                return False, f"Rejected: current mode is {self._mode.value}, not MANUAL"

            if not deadman:
                self._zero_motion_locked()
                return True, "Deadman released - zero velocity applied"

            # Deadman is active: clamp speeds
            clamped_lin = max(-self.max_linear_x, min(self.max_linear_x, float(linear_x)))
            clamped_ang = max(-self.max_angular_z, min(self.max_angular_z, float(angular_z)))

            self._deadman_active = True
            self._last_command_time = time.monotonic()
            self._current_linear_x = clamped_lin
            self._current_angular_z = clamped_ang

        self._emit_velocity(clamped_lin, clamped_ang, True)

        return True, f"Applied (lin={clamped_lin:.2f}, ang={clamped_ang:.2f})"

    def check_watchdog(self) -> bool:
        """Periodic watchdog check to enforce deadman timeout."""
        with self._lock:
            if self._mode == ControlMode.MANUAL:
                now = time.monotonic()
                if (now - self._last_command_time) > self.command_timeout_sec:
                    if self._deadman_active or abs(self._current_linear_x) > 1e-4 or abs(self._current_angular_z) > 1e-4:
                        self._zero_motion_locked()
                        return False  # Timed out and stopped
            return True

    def emergency_stop(self) -> None:
        """Trigger immediate emergency stop and lock in STOPPED mode."""
        with self._lock:
            old_mode = self._mode
            self._zero_motion_locked()
            self._mode = ControlMode.STOPPED

        if self._on_stop_requested:
            self._on_stop_requested()

        if self._on_mode_changed and old_mode != ControlMode.STOPPED:
            self._on_mode_changed(old_mode, ControlMode.STOPPED)

    def handle_disconnect(self) -> None:
        """Browser/client disconnected: immediately stop."""
        self.emergency_stop()

    def _zero_motion_locked(self) -> None:
        """Zero velocity while holding self._lock."""
        self._deadman_active = False
        self._current_linear_x = 0.0
        self._current_angular_z = 0.0
        self._emit_velocity(0.0, 0.0, False)
