"""Unit tests for ControlSupervisor safety boundaries and motion gating."""

import time
import unittest

from cogproj_dashboard.supervisor import ControlMode, ControlSupervisor


class TestControlSupervisor(unittest.TestCase):
    """Test suite verifying safety constraints of ControlSupervisor."""

    def setUp(self):
        self.output_velocities = []
        self.mode_changes = []
        self.stop_requested_count = 0

        self.supervisor = ControlSupervisor(
            max_linear_x=0.35,
            max_angular_z=0.80,
            command_timeout_sec=0.15,
            on_velocity_output=lambda lin, ang: self.output_velocities.append((lin, ang)),
            on_mode_changed=lambda old, new: self.mode_changes.append((old, new)),
            on_stop_requested=lambda: setattr(self, "stop_requested_count", self.stop_requested_count + 1),
        )

    def test_default_mode_is_stopped(self):
        """Supervisor must strictly initialize in STOPPED state with zero velocity."""
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)
        self.assertFalse(self.supervisor.is_moving)

    def test_mode_transitions(self):
        """Mode switches must be explicit and valid."""
        ok, msg = self.supervisor.set_mode("MANUAL")
        self.assertTrue(ok)
        self.assertEqual(self.supervisor.mode, ControlMode.MANUAL)

        ok, msg = self.supervisor.set_mode("AUTONOMOUS")
        self.assertTrue(ok)
        self.assertEqual(self.supervisor.mode, ControlMode.AUTONOMOUS)

        ok, msg = self.supervisor.set_mode("STOPPED")
        self.assertTrue(ok)
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)

        # Invalid mode rejected
        ok, msg = self.supervisor.set_mode("SUPER_SPEED")
        self.assertFalse(ok)
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)

    def test_teleop_rejected_when_not_manual(self):
        """Commands sent during STOPPED or AUTONOMOUS must be strictly rejected."""
        # In STOPPED mode
        ok, msg = self.supervisor.receive_teleop_command(0.08, 0.10, deadman=True)
        self.assertFalse(ok)
        self.assertIn("Rejected", msg)

        # In AUTONOMOUS mode
        self.supervisor.set_mode("AUTONOMOUS")
        ok, msg = self.supervisor.receive_teleop_command(0.08, 0.10, deadman=True)
        self.assertFalse(ok)
        self.assertIn("Rejected", msg)

    def test_velocity_clamping(self):
        """Velocities exceeding configured bounds must be clamped."""
        self.supervisor.set_mode("MANUAL")

        # Exceed positive limits (0.50 m/s > 0.35 m/s, 1.0 rad/s > 0.80 rad/s)
        ok, msg = self.supervisor.receive_teleop_command(0.50, 1.0, deadman=True)
        self.assertTrue(ok)
        last_lin, last_ang = self.output_velocities[-1]
        self.assertAlmostEqual(last_lin, 0.35)
        self.assertAlmostEqual(last_ang, 0.80)

        # Exceed negative limits (-0.50 m/s < -0.35 m/s, -1.0 rad/s < -0.80 rad/s)
        ok, msg = self.supervisor.receive_teleop_command(-0.50, -1.0, deadman=True)
        self.assertTrue(ok)
        last_lin, last_ang = self.output_velocities[-1]
        self.assertAlmostEqual(last_lin, -0.35)
        self.assertAlmostEqual(last_ang, -0.80)

    def test_deadman_switch_release(self):
        """Releasing deadman switch must immediately zero output velocity."""
        self.supervisor.set_mode("MANUAL")
        ok, msg = self.supervisor.receive_teleop_command(0.08, 0.0, deadman=True)
        self.assertTrue(ok)
        self.assertTrue(self.supervisor.is_moving)

        # Release deadman
        ok, msg = self.supervisor.receive_teleop_command(0.08, 0.0, deadman=False)
        self.assertTrue(ok)
        self.assertFalse(self.supervisor.is_moving)
        last_lin, last_ang = self.output_velocities[-1]
        self.assertEqual(last_lin, 0.0)
        self.assertEqual(last_ang, 0.0)

    def test_watchdog_timeout(self):
        """If teleop commands cease without explicit deadman release, watchdog stops vehicle."""
        self.supervisor.set_mode("MANUAL")
        self.supervisor.receive_teleop_command(0.10, 0.0, deadman=True)
        self.assertTrue(self.supervisor.is_moving)

        # Wait past timeout (0.15s + margin)
        time.sleep(0.20)
        ok = self.supervisor.check_watchdog()
        self.assertFalse(ok)  # Timed out
        self.assertFalse(self.supervisor.is_moving)
        last_lin, last_ang = self.output_velocities[-1]
        self.assertEqual(last_lin, 0.0)

    def test_emergency_stop_and_disconnect(self):
        """Emergency stop and client disconnect must immediately lock to STOPPED."""
        self.supervisor.set_mode("MANUAL")
        self.supervisor.receive_teleop_command(0.10, 0.0, deadman=True)
        self.assertTrue(self.supervisor.is_moving)

        self.supervisor.emergency_stop()
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)
        self.assertFalse(self.supervisor.is_moving)
        self.assertEqual(self.stop_requested_count, 1)

        # Test disconnect
        self.supervisor.set_mode("MANUAL")
        self.supervisor.handle_disconnect()
        self.assertEqual(self.supervisor.mode, ControlMode.STOPPED)

    def test_deadman_callback_flag(self):
        """Verify on_velocity_output receives deadman=True on motion and False on stop."""
        received = []
        sup = ControlSupervisor(
            on_velocity_output=lambda lin, ang, deadman: received.append((lin, ang, deadman))
        )
        sup.set_mode("MANUAL")
        # On mode switch, zero motion locked emits deadman=False
        self.assertEqual(received[-1], (0.0, 0.0, False))

        sup.receive_teleop_command(0.08, 0.12, deadman=True)
        self.assertEqual(received[-1], (0.08, 0.12, True))

        sup.receive_teleop_command(0.08, 0.12, deadman=False)
        self.assertEqual(received[-1], (0.0, 0.0, False))


if __name__ == "__main__":
    unittest.main()
