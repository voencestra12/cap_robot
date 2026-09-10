import unittest

from cap_robot.shared_zone import (
    classify_shared_zone_entry,
    expand_aabb,
    point_in_aabb,
    segment_intersects_aabb,
)

# 기본 SHARED_ZONE (P0 확정값): x 150~450, y -50~500
BOX = {'x_min': 150.0, 'x_max': 450.0, 'y_min': -50.0, 'y_max': 500.0}


class PointInAabbTest(unittest.TestCase):
    def test_inside(self):
        self.assertTrue(point_in_aabb(300.0, 200.0, BOX))

    def test_on_edge_is_inside(self):
        self.assertTrue(point_in_aabb(150.0, 0.0, BOX))
        self.assertTrue(point_in_aabb(450.0, 500.0, BOX))

    def test_outside_left(self):
        self.assertFalse(point_in_aabb(100.0, 200.0, BOX))

    def test_outside_below(self):
        self.assertFalse(point_in_aabb(300.0, -100.0, BOX))


class SegmentIntersectsAabbTest(unittest.TestCase):
    def test_both_endpoints_inside(self):
        self.assertTrue(segment_intersects_aabb(200, 0, 400, 300, BOX))

    def test_one_endpoint_inside(self):
        self.assertTrue(segment_intersects_aabb(0, 200, 300, 200, BOX))

    def test_straddles_box_horizontally(self):
        # A구역(x=100)에서 B구역(x=600)으로 가로지르면 중앙 공용 구역을 통과
        self.assertTrue(segment_intersects_aabb(100, 200, 600, 200, BOX))

    def test_fully_outside_no_cross(self):
        # 두 점 모두 박스 아래쪽, 박스를 지나지 않음
        self.assertFalse(segment_intersects_aabb(100, -100, 600, -80, BOX))

    def test_parallel_outside(self):
        self.assertFalse(segment_intersects_aabb(0, 600, 800, 600, BOX))

    def test_corner_clip(self):
        # 왼쪽 아래 코너를 스치는 대각선
        self.assertTrue(segment_intersects_aabb(120, -20, 180, -80, BOX))

    def test_near_miss_corner(self):
        # 코너 근처를 지나가지만 닿지 않음
        self.assertFalse(segment_intersects_aabb(100, -60, 140, -100, BOX))

    def test_own_zone_to_own_zone_no_cross(self):
        # B구역 안에서만 움직이면(둘 다 x>450) 공용 구역 안 건드림
        self.assertFalse(segment_intersects_aabb(500, 100, 650, 300, BOX))


class ClassifySharedZoneEntryTest(unittest.TestCase):
    def test_pick_and_place_both_in_own_zones_no_token(self):
        # A구역(왼쪽 밖)에서 집어 A구역에 놓기 — 공용 구역 미접촉
        api, _ = classify_shared_zone_entry(80, 200, 100, 300, BOX)
        self.assertIsNone(api)

    def test_place_into_basket_center_requires_token_before_place(self):
        # B구역에서 집어 바구니(중앙)에 놓기
        api, reason = classify_shared_zone_entry(600, 200, 300, 220, BOX)
        self.assertEqual(api, 'move_to_place')
        self.assertIn('공용', reason)

    def test_pick_inside_shared_zone_requires_token_before_approach(self):
        api, _ = classify_shared_zone_entry(300, 200, 600, 200, BOX)
        self.assertEqual(api, 'move_to_object')

    def test_transit_crossing_zone_requires_token(self):
        # A구역(x=80)에서 B구역(x=620)으로 옮기면 경로가 중앙을 통과
        api, reason = classify_shared_zone_entry(80, 200, 620, 200, BOX)
        self.assertEqual(api, 'move_to_place')
        self.assertIn('경로', reason)

    def test_b_zone_internal_move_no_token(self):
        api, _ = classify_shared_zone_entry(500, 100, 640, 300, BOX)
        self.assertIsNone(api)


class ExpandAabbTest(unittest.TestCase):
    def test_expand_widens_all_sides(self):
        big = expand_aabb(BOX, 50.0)
        self.assertEqual(big['x_min'], 100.0)
        self.assertEqual(big['x_max'], 500.0)
        self.assertEqual(big['y_min'], -100.0)
        self.assertEqual(big['y_max'], 550.0)

    def test_margin_zero_is_noop(self):
        same = expand_aabb(BOX, 0.0)
        self.assertEqual(same, BOX)

    def test_expanded_box_catches_borderline_point(self):
        self.assertFalse(point_in_aabb(120.0, 200.0, BOX))
        self.assertTrue(point_in_aabb(120.0, 200.0, expand_aabb(BOX, 50.0)))


if __name__ == '__main__':
    unittest.main()
