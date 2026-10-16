import unittest

from cap_robot.detection_fusion import cross_model_keep_indices


class CrossModelFusionTest(unittest.TestCase):
    def test_specialist_wins_even_if_candidates_are_not_in_model_order(self):
        candidates = [(1, (2, 2, 102, 102)), (0, (0, 0, 100, 100))]
        self.assertEqual(cross_model_keep_indices(candidates), [0])

    def test_separate_objects_and_same_model_instances_are_preserved(self):
        candidates = [
            (0, (200, 0, 300, 100)),  # 별도 위치의 실제 사과
            (0, (0, 0, 100, 100)),    # 빵을 사과로 오검출
            (1, (0, 0, 100, 100)),
            (1, (10, 0, 110, 100)),   # 같은 모델의 겹친 개체는 유지
        ]
        self.assertEqual(cross_model_keep_indices(candidates), [0, 2, 3])

    def test_partial_overlap_respects_threshold(self):
        candidates = [(0, (0, 0, 100, 100)), (1, (40, 0, 140, 100))]
        self.assertEqual(cross_model_keep_indices(candidates, 0.6), [0, 1])
        self.assertEqual(cross_model_keep_indices(candidates, 0.4), [1])

    def test_highest_priority_of_three_models_wins(self):
        candidates = [(i, (0, 0, 100, 100)) for i in range(3)]
        self.assertEqual(cross_model_keep_indices(candidates), [2])

    def test_empty_and_degenerate_boxes(self):
        self.assertEqual(cross_model_keep_indices([]), [])
        candidates = [(0, (10, 10, 10, 10)), (1, (10, 10, 10, 10))]
        self.assertEqual(cross_model_keep_indices(candidates), [0, 1])

    def test_invalid_threshold(self):
        for threshold in (0, -1, 1.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                cross_model_keep_indices([], threshold)


if __name__ == '__main__':
    unittest.main()
