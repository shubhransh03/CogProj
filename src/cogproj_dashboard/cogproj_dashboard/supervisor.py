"""Control Supervisor for CogProj Operator Console.

Enforces strict safety boundaries and state arbitration:
1. Mutually exclusive control modes: STOPPED (default), MANUAL, and AUTONOMOUS.
2. Initial state is always STOPPED with zero motion and motors disarmed.
3. Explicit operator action required to enable motion.
4. Independent ROS-side command watchdog (0.3s timeout).
5. Velocity clamping (linear <= 0.10 m/s, angular <= 0.25 rad/s) and non-finite value rejection.
6. Session lease arbitration (prevents conflicting concurrent client commands).
7. Complete navigation state tracking across all required lifecycle phases.
"""

from enum import Enum
import math
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple


class ControlMode(str, Enum):
    """Mutually exclusive vehicle operating modes."""
    STOPPED = "STOPPED"
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"


class NavState(str, Enum):
    """Authoritative autonomous navigation states."""
    NAVIGATION_UNAVAILABLE = "NAVIGATION_UNAVAILABLE"
    WAITING_FOR_LOCALIZATION = "WAITING_FOR_LOCALIZATION"
    READY = "READY"
    GOAL_PENDING = "GOAL_PENDING"
    GOAL_ACCEPTED = "GOAL_ACCEPTED"
    NAVIGATING = "NAVIGATING"
    GOAL_REACHED = "GOAL_REACHED"
    GOAL_FAILED = "GOAL_FAILED"
    GOAL_CANCELED = "GOAL_CANCELED"
    STOPPED = "STOPPED"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    MOTION_INHIBITED = "MOTION_INHIBITED"


class ControlSupervisor:
    """ROS-side safety supervisor for vehicle motion commands and mode arbitration."""

    def __init__(
        self,
        max_linear_x: float = 0.10,
        max_angular_z: float = 0.25,
        command_timeout_sec: float = 0.30,
        on_velocity_output: Optional[Callable[[float, float, bool], None]] = None,
        on_mode_changed: Optional[Callable[[ControlMode, ControlMode], None]] = None,
        on_stop_requested: Optional[Callable[[], None]] = None,
        on_motion_enable_changed: Optional[Callable[[bool], None]] = None,
    ) -> None:
        self.max_linear_x = abs(float(max_linear_x))
        self.max_angular_z = abs(float(max_angular_z))
        self.command_timeout_sec = max(0.05, float(command_timeout_sec))

        # Callbacks
        # on_velocity_output receives (linear_x, angular_z, deadman_pressed)
        self._on_velocity_output = on_velocity_output
        self._on_mode_changed = on_mode_changed
        self._on_stop_requested = on_stop_requested
        self._on_motion_enable_changed = on_motion_enable_changed

        self._lock = threading.RLock()
        
        # State
        self._mode: ControlMode = ControlMode.STOPPED
        self._motion_enabled: bool = False
        self._deadman_active: bool = False
        self._last_command_time: float = 0.0
        self._current_linear_x: float = 0.0
        self._current_angular_z: float = 0.0
        
        # Session lease for single active controller
        self._active_session_id: Optional[str] = None
        
        # Navigation state
        self._nav_state: NavState = NavState.STOPPED
        self._nav_blockers: List[str] = []

    # =========================================================================
    # Properties
    # =========================================================================
    @property
    def mode(self) -> ControlMode:
        with self._lock:
            return self._mode

    @property
    def motion_enabled(self) -> bool:
        with self._lock:
            return self._motion_enabled

    @property
    def is_moving(self) -> bool:
        with self._lock:
            return abs(self._current_linear_x) > 1e-4 or abs(self._current_angular_z) > 1e-4

    @property
    def nav_state(self) -> NavState:
        with self._lock:
            return self._nav_state

    @property
    def active_session_id(self) -> Optional[str]:
        with self._lock:
            return self._active_session_id

    # =========================================================================
    # Session Lease Management
    # =========================================================================
    def acquire_session(self, session_id: str) -> bool:
        """Acquire control session lease if unassigned or matching."""
        if not session_id:
            return False
        with self._lock:
            if self._active_session_id is None or self._active_session_id == session_id:
                self._active_session_id = session_id
                return True
            return False

    def release_session(self, session_id: str) -> None:
        """Release control session lease."""
        with self._lock:
            if self._active_session_id == session_id:
                self._active_session_id = None
                self._zero_motion_locked()
                self._mode = ControlMode.STOPPED
                self._motion_enabled = False
                self._nav_state = NavState.STOPPED

    # =========================================================================
    # Mode Transitions
    # =========================================================================
    def set_mode(self, new_mode_str: str, session_id: Optional[str] = None) -> Tuple[bool, str]:
        """Transition control mode with safe state reset and prerequisite checks."""
        try:
            target_mode = ControlMode(new_mode_str.upper())
        except ValueError:
            return False, f"Invalid mode: '{new_mode_str}'. Must be STOPPED, MANUAL, or AUTONOMOUS."

        with self._lock:
            # Session verification
            if session_id and self._active_session_id and self._active_session_id != session_id:
                return False, "Rejected: another operator session holds the active control lease."

            old_mode = self._mode
            if old_mode == target_mode:
                return True, f"Mode already {target_mode.value}"

            # Always zero motion and cancel active commands on any transition
            self._zero_motion_locked()

            # Entering STOPPED
            if target_mode == ControlMode.STOPPED:
                self._mode = ControlMode.STOPPED
                self._motion_enabled = False
                self._nav_state = NavState.STOPPED
                if session_id:
                    self._active_session_id = session_id

            # Entering MANUAL
            elif target_mode == ControlMode.MANUAL:
                self._mode = ControlMode.MANUAL
                self._motion_enabled = True  # Selecting MANUAL enables motion with keypress deadman
                self._nav_state = NavState.READY
                if session_id:
                    self._active_session_id = session_id

            # Entering AUTONOMOUS
            elif target_mode == ControlMode.AUTONOMOUS:
                self._mode = ControlMode.AUTONOMOUS
                self._motion_enabled = True  # Selecting AUTONOMOUS enables motion
                self._nav_state = NavState.READY
                if session_id:
                    self._active_session_id = session_id

        # Notify callbacks outside lock
        if self._on_mode_changed:
            self._on_mode_changed(old_mode, target_mode)

        return True, f"Mode switched from {old_mode.value} to {target_mode.value}"

    def enable_motion(self, enabled: bool, session_id: Optional[str] = None) -> Tuple[bool, str]:
        """Explicit operator action to arm/enable or disarm/disable motion."""
        with self._lock:
            if session_id and self._active_session_id and self._active_session_id != session_id:
                return False, "Rejected: another operator session holds the control lease."

            if self._mode == ControlMode.STOPPED:
                self._motion_enabled = False
                return False, "Cannot enable motion while in STOPPED mode. Select MANUAL or AUTONOMOUS first."

            self._motion_enabled = bool(enabled)
            if not self._motion_enabled:
                self._zero_motion_locked()
                if self._mode == ControlMode.AUTONOMOUS:
                    self._nav_state = NavState.MOTION_INHIBITED
            else:
                if self._mode == ControlMode.AUTONOMOUS and self._nav_state == NavState.MOTION_INHIBITED:
                    self._nav_state = NavState.READY

        if self._on_motion_enable_changed:
            self._on_motion_enable_changed(self._motion_enabled)

        state_str = "ENABLED" if enabled else "DISABLED"
        return True, f"Motion {state_str} for {self._mode.value} mode"

    # =========================================================================
    # Teleoperation Commands
    # =========================================================================
    def receive_teleop_command(
        self, linear_x: Any, angular_z: Any, deadman: bool, session_id: Optional[str] = None
    ) -> Tuple[bool, str]:
        """Validate, clamp, and process manual keyboard commands with complete safety checks."""
        # 1. Type and finiteness validation
        try:
            lin = float(linear_x)
            ang = float(angular_z)
        except (ValueError, TypeError):
            with self._lock:
                self._zero_motion_locked()
            return False, "Rejected: non-numeric velocity values received."

        if not (math.isfinite(lin) and math.isfinite(ang)):
            with self._lock:
                self._zero_motion_locked()
            return False, "Rejected: NaN or Infinite velocity values received."

        with self._lock:
            # 2. Session check
            if session_id and self._active_session_id and self._active_session_id != session_id:
                return False, "Rejected: session does not hold active control lease."

            # 3. Mode check
            if self._mode != ControlMode.MANUAL:
                self._zero_motion_locked()
                return False, f"Rejected: current mode is {self._mode.value}, not MANUAL."

            # 4. Motion enablement check
            if not self._motion_enabled:
                self._zero_motion_locked()
                return False, "Rejected: motion is disabled. Operator must enable motion first."

            # 5. Deadman release check
            if not deadman or (abs(lin) < 1e-4 and abs(ang) < 1e-4):
                self._zero_motion_locked()
                return True, "Stop/Deadman released: zero velocity applied."

            # 6. Server-side velocity clamping
            clamped_lin = max(-self.max_linear_x, min(self.max_linear_x, lin))
            clamped_ang = max(-self.max_angular_z, min(self.max_angular_z, ang))

            self._deadman_active = True
            self._last_command_time = time.monotonic()
            self._current_linear_x = clamped_lin
            self._current_angular_z = clamped_ang

        # Output to pre-gate topic
        if self._on_velocity_output:
            self._on_velocity_output(clamped_lin, clamped_ang, True)

        return True, f"Command applied: lin={clamped_lin:.2f} m/s, ang={clamped_ang:.2f} rad/s"

    # =========================================================================
    # Watchdog & Emergency Stop
    # =========================================================================
    def check_watchdog(self) -> bool:
        """Periodic ROS watchdog check (20 Hz) to enforce deadman timeout."""
        with self._lock:
            if self._mode == ControlMode.MANUAL and self._deadman_active:
                now = time.monotonic()
                if (now - self._last_command_time) > self.command_timeout_sec:
                    # Lease expired! Auto-stop immediately.
                    self._zero_motion_locked()
                    return False
            return True

    def emergency_stop(self) -> None:
        """Trigger immediate emergency stop and lock in STOPPED mode."""
        with self._lock:
            old_mode = self._mode
            self._zero_motion_locked()
            self._mode = ControlMode.STOPPED
            self._motion_enabled = False
            self._nav_state = NavState.EMERGENCY_STOP

        if self._on_stop_requested:
            self._on_stop_requested()

        if self._on_mode_changed and old_mode != ControlMode.STOPPED:
            self._on_mode_changed(old_mode, ControlMode.STOPPED)

    def handle_disconnect(self, session_id: Optional[str] = None) -> None:
        """Client disconnected: immediately stop vehicle motion."""
        with self._lock:
            if session_id and self._active_session_id and self._active_session_id != session_id:
                return  # Observer disconnected, active controller unaffected
            self._active_session_id = None

        self.emergency_stop()

    def set_nav_state(self, state: NavState, blockers: Optional[List[str]] = None) -> None:
        """Update autonomous navigation state and active blockers."""
        with self._lock:
            self._nav_state = state
            if blockers is not None:
                self._nav_blockers = list(blockers)

    def get_status_dict(self) -> Dict[str, Any]:
        """Generate a thread-safe snapshot of supervisor status."""
        with self._lock:
            now = time.monotonic()
            watchdog_age = round((now - self._last_command_time) * 1000, 1) if self._last_command_time > 0 else 9999.0
            watchdog_healthy = (now - self._last_command_time) <= self.command_timeout_sec if self._deadman_active else True

            return {
                "mode": self._mode.value,
                "motion_enabled": self._motion_enabled,
                "is_moving": self.is_moving,
                "linear_velocity": round(self._current_linear_x, 3),
                "angular_velocity": round(self._current_angular_z, 3),
                "deadman_active": self._deadman_active,
                "watchdog_healthy": watchdog_healthy,
                "watchdog_age_ms": watchdog_age,
                "active_session": self._active_session_id is not None,
                "nav_state": self._nav_state.value,
                "nav_blockers": list(self._nav_blockers),
            }

    # =========================================================================
    # Internal Helpers
    # =========================================================================
    def _zero_motion_locked(self) -> None:
        """Zero velocity and de-assert deadman while holding lock."""
        self._deadman_active = False
        self._current_linear_x = 0.0
        self._current_angular_z = 0.0
        if self._on_velocity_output:
            self._on_velocity_output(0.0, 0.0, False)
