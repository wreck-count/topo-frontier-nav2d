import glob
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot import load_recording

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), 'fixtures')


def _fixture_paths():
    return sorted(glob.glob(os.path.join(FIXTURES_DIR, '*.json')))


@pytest.mark.parametrize('fixture_path', _fixture_paths(), ids=lambda p: os.path.basename(p))
def test_replay_recording(fixture_path):
    maze, robot = load_recording(fixture_path)
    poly = robot.mapping.polygon
    assert poly.is_valid
