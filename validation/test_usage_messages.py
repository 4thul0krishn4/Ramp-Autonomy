#!/usr/bin/env python3
"""Check that validation scripts report their required arguments."""
import subprocess
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CASES = {
    'phase8/construction_site_descent/crosstrack_diagnosis_flatmode/'
    'drift_characterization/analyze_highrate.py': (
        ('highrate.csv', 'trial_row.csv'),
        'usage: analyze_highrate.py <highrate.csv> <trial_row.csv>',
    ),
    'phase8/construction_site_descent/crosstrack_diagnosis_flatmode/'
    'drift_characterization/analyze_scan.py': (
        ('scan.npz', 'trial_row.csv'),
        'usage: analyze_scan.py <scan.npz> <trial_row.csv>',
    ),
    'phase8/corner_mount/obstacle_lidar_check_corner.py': (
        ('out.json',),
        'usage: obstacle_lidar_check_corner.py <out.json>',
    ),
    'phase8/crest_freeze/crest_diag.py': (
        ('label', 'world', 'out.npz'),
        'usage: crest_diag.py <label> <gz_world_name> <out.npz>',
    ),
    'phase8/delivery/analyze_rtf_states.py': (
        ('runs',),
        'usage: analyze_rtf_states.py <dir>',
    ),
    'phase8/delivery/full_mission_test.py': (
        ('out.json',),
        'usage: full_mission_test.py <out.json>',
    ),
}


class UsageMessageTest(unittest.TestCase):
    def test_missing_arguments_show_usage_without_a_traceback(self):
        for relative_path, (arguments, usage) in CASES.items():
            for supplied in range(len(arguments)):
                with self.subTest(script=relative_path, supplied=supplied):
                    result = subprocess.run(
                        [sys.executable, str(ROOT / relative_path),
                         *arguments[:supplied]],
                        capture_output=True,
                        text=True,
                        check=False,
                    )
                    self.assertNotEqual(result.returncode, 0)
                    self.assertEqual(result.stdout, '')
                    self.assertEqual(result.stderr.strip(), usage)
                    self.assertNotIn('Traceback', result.stderr)


if __name__ == '__main__':
    unittest.main()
