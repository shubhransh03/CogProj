"""Comprehensive unit tests for ControlSupervisor safety boundaries and motion gating (Phase 5M)."""

import math
import time
import unittest

from cogproj_dashboard.supervisor import ControlMode, ControlSupervisor, NavState


class TestControlSupervisor(unittest.TestCase):
    """Test suite verifying safety constraints, mode arbitration, and leases in ControlSupervisor."""

    def setUp(self):
        self.output_velocities = []
        self.mode_changes = []
        self.stop_requested_count = 0
        self.motion_enable_changes = []

        self.supervisor = ControlSupervisor(
            max_linear_x=0.10,
            max_angular_z=0.25,
            command_timeout_sec=0.15,
            on_velocity_output=lambda lin, ang, deadman: self.output_velocities.append((lin, ang, deadman)),
            on_mode_changed=lambda old, new: self.mode_changes.append((old, new)),
            on_stop_requested=lambda: setattr(self, "stop_requested_count", self.stop_requested_count + 1),
            on_motion_enable_changed=lambda enabled: self.motion_enable_changes.append(enabled),
        )

    def test_default_mode_is_stopped_with_zero_motion(self):
        """Supervisor must strictly initialize in STOPPED state with zero velocity and motion disabled."""
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)
        self.assertFalse(self.supervisor.motion_enabled)
        self.assertFalse(self.supervisor.is_moving)
        self.assertEqual(self.supervisor.nav_state, NavState.STOPPED)

    def test_mode_transitions(self):
        """Mode switches must be explicit, valid, and zero any prior movement."""
        # STOPPED -> MANUAL
        ok, msg = self.supervisor.set_mode("MANUAL")
        self.assertTrue(ok)
        self.assertEqual(self.supervisor.mode, ControlMode.MANUAL)
        self.assertTrue(self.supervisor.motion_enabled, "Entering MANUAL must enable motion with keypress deadman")

        # MANUAL -> AUTONOMOUS
        ok, msg = self.supervisor.set_mode("AUTONOMOUS")
        self.assertTrue(ok)
        self.assertEqual(self.supervisor.mode, ControlMode.AUTONOMOUS)
        self.assertTrue(self.supervisor.motion_enabled)

        # AUTONOMOUS -> STOPPED
        ok, msg = self.supervisor.set_mode("STOPPED")
        self.assertTrue(ok)
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)
        self.assertFalse(self.supervisor.motion_enabled)

        # Invalid mode string rejected
        ok, msg = self.supervisor.set_mode("TURBO_MODE")
        self.assertFalse(ok)
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)

    def test_motion_enablement_gating(self):
        """Motion enable toggle must be strictly gated and require non-STOPPED mode."""
        # Attempt to enable motion while STOPPED -> rejected
        ok, msg = self.supervisor.enable_motion(True)
        self.assertFalse(ok)
        self.assertFalse(self.supervisor.motion_enabled)

        # Switch to MANUAL -> initially enabled by mode entry
        self.supervisor.set_mode("MANUAL")
        self.assertTrue(self.supervisor.motion_enabled)

        # Operator explicitly disables motion
        ok, msg = self.supervisor.enable_motion(False)
        self.assertTrue(ok)
        self.assertFalse(self.supervisor.motion_enabled)
        self.assertEqual(self.motion_enable_changes[-1], False)

        # Operator explicitly re-enables motion
        ok, msg = self.supervisor.enable_motion(True)
        self.assertTrue(ok)
        self.assertTrue(self.supervisor.motion_enabled)
        self.assertEqual(self.motion_enable_changes[-1], True)

    def test_teleop_rejected_when_not_manual(self):
        """Commands sent during STOPPED or AUTONOMOUS must be strictly rejected."""
        # In STOPPED mode
        ok, msg = self.supervisor.receive_teleop_command(0.08, 0.10, deadman=True)
        self.assertFalse(ok)
        self.assertIn("Rejected", msg)

        # In AUTONOMOUS mode
        self.supervisor.set_mode("AUTONOMOUS")
        self.supervisor.enable_motion(True)
        ok, msg = self.supervisor.receive_teleop_command(0.08, 0.10, deadman=True)
        self.assertFalse(ok)
        self.assertIn("Rejected", msg)

    def test_teleop_rejected_when_motion_disabled(self):
        """Teleop commands in MANUAL must be rejected if operator explicitly disabled motion."""
        self.supervisor.set_mode("MANUAL")
        self.supervisor.enable_motion(False)
        self.assertFalse(self.supervisor.motion_enabled)

        ok, msg = self.supervisor.receive_teleop_command(0.08, 0.10, deadman=True)
        self.assertFalse(ok)
        self.assertIn("Rejected: motion is disabled", msg)
        self.assertFalse(self.supervisor.is_moving)

    def test_teleop_rejected_on_non_finite_and_malformed_values(self):
        """NaN, Infinite, or non-numeric velocities must be rejected and immediately zero motion."""
        self.supervisor.set_mode("MANUAL")
        self.supervisor.enable_motion(True)

        # NaN linear velocity
        ok, msg = self.supervisor.receive_teleop_command(float("nan"), 0.10, deadman=True)
        self.assertFalse(ok)
        self.assertIn("NaN or Infinite", msg)
        self.assertFalse(self.supervisor.is_moving)

        # Infinite angular velocity
        ok, msg = self.supervisor.receive_teleop_command(0.05, float("inf"), deadman=True)
        self.assertFalse(ok)
        self.assertIn("NaN or Infinite", msg)

        # String / non-numeric
        ok, msg = self.supervisor.receive_teleop_command("fast", 0.0, deadman=True)
        self.assertFalse(ok)
        self.assertIn("non-numeric", msg)

    def test_velocity_clamping(self):
        """Velocities exceeding configured bounds must be clamped strictly on the server."""
        self.supervisor.set_mode("MANUAL")
        self.supervisor.enable_motion(True)

        # Exceed positive limits (0.50 m/s > 0.10 m/s, 1.0 rad/s > 0.25 rad/s)
        ok, msg = self.supervisor.receive_teleop_command(0.50, 1.0, deadman=True)
        self.assertTrue(ok)
        last_lin, last_ang, deadman = self.output_velocities[-1]
        self.assertAlmostEqual(last_lin, 0.10)
        self.assertAlmostEqual(last_ang, 0.25)
        self.assertTrue(deadman)

        # Exceed negative limits (-0.50 m/s < -0.10 m/s, -1.0 rad/s < -0.25 rad/s)
        ok, msg = self.supervisor.receive_teleop_command(-0.50, -1.0, deadman=True)
        self.assertTrue(ok)
        last_lin, last_ang, deadman = self.output_velocities[-1]
        self.assertAlmostEqual(last_lin, -0.10)
        self.assertAlmostEqual(last_ang, -0.25)
        self.assertTrue(deadman)

    def test_deadman_switch_release(self):
        """Releasing deadman switch must immediately zero output velocity and de-assert deadman."""
        self.supervisor.set_mode("MANUAL")
        self.supervisor.enable_motion(True)

        ok, msg = self.supervisor.receive_teleop_command(0.08, 0.0, deadman=True)
        self.assertTrue(ok)
        self.assertTrue(self.supervisor.is_moving)

        # Release deadman
        ok, msg = self.supervisor.receive_teleop_command(0.08, 0.0, deadman=False)
        self.assertTrue(ok)
        self.assertFalse(self.supervisor.is_moving)
        last_lin, last_ang, deadman = self.output_velocities[-1]
        self.assertEqual(last_lin, 0.0)
        self.assertEqual(last_ang, 0.0)
        self.assertFalse(deadman)

    def test_watchdog_timeout(self):
        """If teleop commands cease without explicit deadman release, watchdog stops vehicle."""
        self.supervisor.set_mode("MANUAL")
        self.supervisor.enable_motion(True)
        self.supervisor.receive_teleop_command(0.10, 0.0, deadman=True)
        self.assertTrue(self.supervisor.is_moving)

        # Wait past timeout (0.15s + margin)
        time.sleep(0.20)
        ok = self.supervisor.check_watchdog()
        self.assertFalse(ok, "Watchdog must report expired")
        self.assertFalse(self.supervisor.is_moving)
        last_lin, last_ang, deadman = self.output_velocities[-1]
        self.assertEqual(last_lin, 0.0)
        self.assertEqual(last_ang, 0.0)
        self.assertFalse(deadman)

    def test_emergency_stop_and_disconnect(self):
        """Emergency stop and client disconnect must immediately lock to STOPPED."""
        self.supervisor.set_mode("MANUAL")
        self.supervisor.enable_motion(True)
        self.supervisor.receive_teleop_command(0.10, 0.0, deadman=True)
        self.assertTrue(self.supervisor.is_moving)

        self.supervisor.emergency_stop()
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)
        self.assertFalse(self.supervisor.motion_enabled)
        self.assertFalse(self.supervisor.is_moving)
        self.assertEqual(self.supervisor.nav_state, NavState.EMERGENCY_STOP)
        self.assertEqual(self.stop_requested_count, 1)

        # Disconnect test
        self.supervisor.set_mode("MANUAL")
        self.supervisor.handle_disconnect()
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)

    def test_session_lease_arbitration(self):
        """Concurrent or secondary browser sessions cannot hijack control without lease."""
        sess_a = "session_alpha"
        sess_b = "session_beta"

        # Session A enters MANUAL
        ok, msg = self.supervisor.set_mode("MANUAL", session_id=sess_a)
        self.assertTrue(ok)
        self.supervisor.enable_motion(True, session_id=sess_a)

        # Session B attempts to send teleop -> rejected
        ok, msg = self.supervisor.receive_teleop_command(0.10, 0.0, deadman=True, session_id=sess_b)
        self.assertFalse(ok)
        self.assertIn("lease", msg)

        # Session A teleop succeeds
        ok, msg = self.supervisor.receive_teleop_command(0.10, 0.0, deadman=True, session_id=sess_a)
        self.assertTrue(ok)

        # Session A disconnects / releases lease
        self.supervisor.release_session(sess_a)
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)

        # Session B now acquires control in AUTONOMOUS
        ok, msg = self.supervisor.set_mode("AUTONOMOUS", session_id=sess_b)
        self.assertTrue(ok)
        self.assertEqual(self.supervisor.mode, ControlMode.AUTONOMOUS)

    def test_status_dict_completeness(self):
        """get_status_dict must provide full snapshot of control states."""
        status = self.supervisor.get_status_dict()
        required_keys = [
            "mode",
            "motion_enabled",
            "is_moving",
            "linear_velocity",
            "angular_velocity",
            "deadman_active",
            "watchdog_healthy",
            "watchdog_age_ms",
            "active_session",
            "nav_state",
            "nav_blockers",
        ]
        for k in required_keys:
            self.assertIn(k, status, f"Missing key {k} in get_status_dict")


if __name__ == "__main__":
    unittest.main()
