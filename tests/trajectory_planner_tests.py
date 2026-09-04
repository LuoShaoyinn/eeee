#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from trajectory_planner_sim import (  # noqa: E402
    FRONT_MIDPOINT,
    Pose,
    Target,
    TargetLock,
    path_safe,
    plan_collection,
    rotation,
    safe,
)


class TrajectoryCases(unittest.TestCase):
    margin = .10
    start = Pose(1.5, 1.0, 0.0)

    def check_plan(self, target: tuple[float, float]) -> list[Pose]:
        point = np.asarray(target)
        path = plan_collection(self.start, point, self.margin)
        self.assertIsNotNone(path)
        assert path is not None
        self.assertTrue(path_safe(path, self.margin))
        relative = rotation(path[-1].yaw).T @ (point - (path[-1].x, path[-1].y))
        np.testing.assert_allclose(relative, FRONT_MIDPOINT, atol=1e-9)
        return path

    def test_open_arena(self) -> None:
        self.check_plan((2.2, 1.1))

    def test_left_fence_requires_outward_facing_front(self) -> None:
        path = self.check_plan((.18, 1.0))
        self.assertLess(math.cos(path[-1].yaw), -.8)

    def test_right_fence_requires_outward_facing_front(self) -> None:
        path = self.check_plan((2.82, 1.0))
        self.assertGreater(math.cos(path[-1].yaw), .8)

    def test_bottom_fence_requires_outward_facing_front(self) -> None:
        path = self.check_plan((1.5, .18))
        self.assertLess(math.sin(path[-1].yaw), -.8)

    def test_top_fence_requires_outward_facing_front(self) -> None:
        path = self.check_plan((1.5, 1.805))
        self.assertGreater(math.sin(path[-1].yaw), .8)

    def test_corner_collection(self) -> None:
        path = self.check_plan((2.82, 1.75))
        self.assertGreater(math.cos(path[-1].yaw), 0)
        self.assertGreater(math.sin(path[-1].yaw), 0)

    def test_invalid_start_is_rejected(self) -> None:
        start = Pose(.20, .20, 0.0)
        self.assertFalse(safe(start, self.margin))
        self.assertIsNone(plan_collection(start, np.array([1.0, 1.0]), self.margin))


class TargetLockCases(unittest.TestCase):
    def test_challenger_does_not_interrupt(self) -> None:
        lock = TargetLock()
        self.assertEqual(lock.update([Target(1, 1, 1)], 0).track_id, 1)
        self.assertEqual(lock.update([Target(2, .5, .5)], .1).track_id, 1)
        self.assertEqual(lock.update([Target(1, 1, 1), Target(2, .5, .5)], .2).track_id, 1)

    def test_changed_detector_id_is_spatially_reacquired(self) -> None:
        lock = TargetLock()
        lock.update([Target(1, 1, 1)], 0)
        target = lock.update([Target(99, 1.04, .98)], .1)
        self.assertEqual(target.track_id, 1)
        self.assertAlmostEqual(target.x, 1.04)

    def test_normal_loss_is_bounded(self) -> None:
        lock = TargetLock()
        lock.update([Target(1, 1, 1)], 0)
        self.assertIsNotNone(lock.update([], .7))
        self.assertIsNone(lock.update([], .76))

    def test_terminal_loss_has_longer_but_bounded_grace(self) -> None:
        lock = TargetLock()
        lock.update([Target(1, 1, 1)], 0)
        self.assertIsNotNone(lock.update([], 1.4, terminal_approach=True))
        self.assertIsNone(lock.update([], 1.51, terminal_approach=True))

    def test_completion_allows_next_target(self) -> None:
        lock = TargetLock()
        lock.update([Target(1, 1, 1)], 0)
        lock.complete_or_abandon()
        self.assertEqual(lock.update([Target(2, .5, .5)], .1).track_id, 2)


if __name__ == "__main__":
    unittest.main()
