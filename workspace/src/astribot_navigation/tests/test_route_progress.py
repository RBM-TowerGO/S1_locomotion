from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from real_navigation.core import RouteProgress


def test_failed_tracking_invalidates_cached_route_before_delay():
    tree = ET.parse(ROOT / 'scripts/real_navigation/navigation.xml')
    fallback = tree.find('.//PipelineSequence/..')
    assert fallback[0].tag == 'PipelineSequence'
    invalidate, delay = fallback[1]
    assert invalidate.attrib == {'output_key': 'route_kind', 'value': 'replan'}
    assert delay.find('AlwaysFailure') is not None


def test_no_progress_survives_pauses_and_ignores_stationary_jitter():
    progress = RouteProgress(.3)
    assert not progress.stalled(0, [1., 2.])
    assert not progress.stalled(30, [1.02, 2.01])
    progress.pause()
    assert not progress.stalled(180, [1., 2.])
    assert progress.stalled(211, [1., 2.])


def test_slow_sideways_motion_and_final_turn_are_progress():
    slow = RouteProgress(.0005)
    for tick in range(600):
        assert not slow.stalled(tick, [0., tick * .0005])
    turning = RouteProgress(.3)
    for tick in range(100):
        assert not turning.stalled(tick, [0., 0.], 2. - tick * .01)
