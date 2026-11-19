#!/usr/bin/env python3
"""Calibration must reject motion and compensated input, not just fit any mean."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('chassis_imu_calibrate', ROOT/'tools/chassis_imu_calibrate.py')
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw = json.loads((ROOT/'tests/firmware/2026-11-18_imu_stationary_level.json').read_text())

    def setUp(self):
        self.data = copy.deepcopy(self.raw)
        # Full historical window has a short disturbance around t=27s.
        self.data["imu_samples"] = [s for s in self.data["imu_samples"] if s["t"] <= 26]

    def test_known_stationary_bias(self):
        result = calibration.estimate(self.data)
        self.assertEqual(result['samples'], 1050)
        expected = [-0.013824510969931172, -0.00503225992699819, 0.024554587195494346]
        for measured, reference in zip(result['gyro_bias_rad_s'], expected):
            self.assertAlmostEqual(measured, reference, places=10)
        header = calibration.render_header(result, 'source-sha')
        self.assertIn('CHASSIS_IMU_BIAS_ENABLED 1', header)
        self.assertIn('raw - bias', header)

    def test_historical_disturbance(self):
        with self.assertRaises(ValueError): calibration.estimate(self.raw)

    def test_motion_transient(self):
        self.data['imu_samples'][-100]['gyro'][0] += 0.2
        with self.assertRaises(ValueError): calibration.estimate(self.data)

    def test_small_sustained_rotation_is_not_observable(self):
        # This passes numerical gates: independent stationary confirmation matters.
        for sample in self.data['imu_samples']: sample['gyro'][2] += 0.01
        result = calibration.estimate(self.data)
        self.assertAlmostEqual(result['gyro_bias_rad_s'][2], 0.03455458719549435)

    def test_compensated_data(self):
        self.data['diagnostics'][-1]['data']['imu_bias_enabled'] = 1
        with self.assertRaises(ValueError): calibration.estimate(self.data)

    def test_motor_command(self):
        self.data['diagnostics'][-1]['data']['enabled'] = 1
        with self.assertRaises(ValueError): calibration.estimate(self.data)

    def test_wheel_motion(self):
        self.data['diagnostics'][-1]['data']['counts'][3] += 1
        with self.assertRaises(ValueError): calibration.estimate(self.data)

    def test_publisher_appeared_during_capture(self):
        self.data['summary']['cmd_vel_publishers_seen'] = ['unexpected_controller']
        with self.assertRaises(ValueError): calibration.estimate(self.data)

    def test_clock_jump(self):
        self.data['imu_samples'][-100]['stamp'] -= 1
        with self.assertRaises(ValueError): calibration.estimate(self.data)

    def test_imu_failure(self):
        self.data['diagnostics'][-1]['data']['imu_errors'] = 1
        with self.assertRaises(ValueError): calibration.estimate(self.data)

    def test_short_capture(self):
        self.data['imu_samples'] = self.data['imu_samples'][-100:]
        with self.assertRaises(ValueError): calibration.estimate(self.data)


if __name__ == '__main__':
    unittest.main()
