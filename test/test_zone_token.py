import unittest

from cap_robot.zone_token import (
    GRANTED,
    IGNORED,
    QUEUED,
    RELEASED,
    ZoneTokenCore,
)


class ZoneTokenCoreTest(unittest.TestCase):
    def setUp(self):
        self.core = ZoneTokenCore(lease_sec=30.0)

    # --- 기본 획득/대기 ------------------------------------------------
    def test_acquire_on_free_zone_grants_and_bumps_epoch(self):
        result, epoch = self.core.acquire('agent1', now=0.0)
        self.assertEqual(result, GRANTED)
        self.assertEqual(epoch, 1)
        self.assertEqual(self.core.holder, 'agent1')
        self.assertEqual(self.core.lease_deadline, 30.0)

    def test_second_agent_is_queued(self):
        self.core.acquire('agent1', now=0.0)
        result, epoch = self.core.acquire('agent2', now=1.0)
        self.assertEqual(result, QUEUED)
        self.assertIsNone(epoch)
        self.assertEqual(self.core.holder, 'agent1')
        self.assertEqual(self.core.waiters, ['agent2'])

    def test_duplicate_queue_request_is_not_added_twice(self):
        self.core.acquire('agent1', now=0.0)
        self.core.acquire('agent2', now=1.0)
        self.core.acquire('agent2', now=2.0)
        self.assertEqual(self.core.waiters, ['agent2'])

    # --- 반납 / 공정 큐 ---------------------------------------------------
    def test_release_hands_off_to_waiter_with_new_epoch(self):
        _, epoch1 = self.core.acquire('agent1', now=0.0)
        self.core.acquire('agent2', now=1.0)
        result, reason = self.core.release('agent1', epoch=epoch1, now=2.0)
        self.assertEqual(result, RELEASED)
        self.assertIsNone(reason)
        self.assertEqual(self.core.holder, 'agent2')
        self.assertEqual(self.core.waiters, [])
        self.assertEqual(self.core.epoch, 2)
        self.assertEqual(self.core.lease_deadline, 32.0)

    def test_releaser_goes_behind_waiting_peer(self):
        _, epoch1 = self.core.acquire('agent1', now=0.0)
        self.core.acquire('agent2', now=1.0)
        self.core.release('agent1', epoch=epoch1, now=2.0)
        # agent1 이 곧바로 다시 요청해도 즉시 획득하지 못하고 큐로 간다.
        result, _ = self.core.acquire('agent1', now=3.0)
        self.assertEqual(result, QUEUED)
        self.assertEqual(self.core.holder, 'agent2')
        self.assertEqual(self.core.waiters, ['agent1'])

    def test_release_with_empty_queue_frees_zone(self):
        _, epoch1 = self.core.acquire('agent1', now=0.0)
        self.core.release('agent1', epoch=epoch1, now=1.0)
        self.assertIsNone(self.core.holder)
        self.assertIsNone(self.core.lease_deadline)
        # 이제 아무나 즉시 획득 가능
        result, _ = self.core.acquire('agent1', now=2.0)
        self.assertEqual(result, GRANTED)

    # --- 소유권 / epoch 검증 -------------------------------------------
    def test_release_by_non_holder_is_ignored(self):
        self.core.acquire('agent1', now=0.0)
        result, reason = self.core.release('agent2', epoch=1, now=1.0)
        self.assertEqual(result, IGNORED)
        self.assertEqual(reason, 'not_holder')
        self.assertEqual(self.core.holder, 'agent1')

    def test_release_with_stale_epoch_is_ignored(self):
        self.core.acquire('agent1', now=0.0)
        result, reason = self.core.release('agent1', epoch=999, now=1.0)
        self.assertEqual(result, IGNORED)
        self.assertEqual(reason, 'stale_epoch')
        self.assertEqual(self.core.holder, 'agent1')

    def test_release_without_epoch_is_accepted_when_holder_matches(self):
        self.core.acquire('agent1', now=0.0)
        result, _ = self.core.release('agent1', epoch=None, now=1.0)
        self.assertEqual(result, RELEASED)

    def test_late_release_after_reclaim_does_not_free_new_holder(self):
        _, epoch1 = self.core.acquire('agent1', now=0.0)
        self.core.acquire('agent2', now=1.0)
        # agent1 lease 만료 -> agent2 로 회수
        changed, reclaimed = self.core.tick(now=31.0)
        self.assertTrue(changed)
        self.assertEqual(reclaimed, 'agent1')
        self.assertEqual(self.core.holder, 'agent2')
        # agent1 의 지각 반납은 무시되어야 한다.
        result, reason = self.core.release('agent1', epoch=epoch1, now=32.0)
        self.assertEqual(result, IGNORED)
        self.assertEqual(self.core.holder, 'agent2')

    # --- 재진입 / 하트비트 -------------------------------------------
    def test_reentrant_acquire_refreshes_lease_and_keeps_epoch(self):
        _, epoch1 = self.core.acquire('agent1', now=0.0)
        result, epoch2 = self.core.acquire('agent1', now=10.0)
        self.assertEqual(result, GRANTED)
        self.assertEqual(epoch2, epoch1)
        self.assertEqual(self.core.lease_deadline, 40.0)

    def test_heartbeat_via_reentrant_acquire_prevents_reclaim(self):
        # 협동 coordinator가 긴 작업 중 재진입 acquire(하트비트)로 lease를 갱신하면
        # 원래 deadline이 지나도 tick()이 회수하지 않아야 한다.
        self.core.acquire('agent1', now=0.0)          # deadline 30
        self.core.acquire('agent1', now=20.0)         # 하트비트 -> deadline 50
        changed, _ = self.core.tick(now=31.0)         # 하트비트 없었으면 만료됐을 시각
        self.assertFalse(changed)
        self.assertEqual(self.core.holder, 'agent1')
        # 갱신된 deadline(50)을 넘기면 그때는 회수된다.
        changed, reclaimed = self.core.tick(now=50.0)
        self.assertTrue(changed)
        self.assertEqual(reclaimed, 'agent1')

    # --- lease 만료 ---------------------------------------------------
    def test_tick_before_deadline_is_noop(self):
        self.core.acquire('agent1', now=0.0)
        changed, reclaimed = self.core.tick(now=29.9)
        self.assertFalse(changed)
        self.assertIsNone(reclaimed)
        self.assertEqual(self.core.holder, 'agent1')

    def test_tick_after_deadline_without_waiter_frees_zone(self):
        self.core.acquire('agent1', now=0.0)
        changed, reclaimed = self.core.tick(now=30.0)
        self.assertTrue(changed)
        self.assertEqual(reclaimed, 'agent1')
        self.assertIsNone(self.core.holder)
        self.assertIsNone(self.core.lease_deadline)

    def test_tick_on_empty_zone_is_noop(self):
        changed, reclaimed = self.core.tick(now=100.0)
        self.assertFalse(changed)
        self.assertIsNone(reclaimed)

    # --- abandon (liveness 상실) -----------------------------------
    def test_abandon_holder_hands_off(self):
        self.core.acquire('agent1', now=0.0)
        self.core.acquire('agent2', now=1.0)
        changed = self.core.abandon('agent1', now=2.0)
        self.assertTrue(changed)
        self.assertEqual(self.core.holder, 'agent2')

    def test_abandon_waiter_removes_from_queue(self):
        self.core.acquire('agent1', now=0.0)
        self.core.acquire('agent2', now=1.0)
        changed = self.core.abandon('agent2', now=2.0)
        self.assertTrue(changed)
        self.assertEqual(self.core.holder, 'agent1')
        self.assertEqual(self.core.waiters, [])

    def test_abandon_unknown_agent_is_noop(self):
        self.core.acquire('agent1', now=0.0)
        changed = self.core.abandon('agent2', now=1.0)
        self.assertFalse(changed)

    # --- epoch 단조 증가 -------------------------------------------
    def test_epoch_is_monotonic_across_grants(self):
        _, e1 = self.core.acquire('agent1', now=0.0)
        self.core.acquire('agent2', now=1.0)
        self.core.release('agent1', epoch=e1, now=2.0)  # -> agent2, epoch 2
        _, e3 = self.core.acquire('agent1', now=3.0)     # queued
        self.core.release('agent2', epoch=2, now=4.0)    # -> agent1, epoch 3
        self.assertEqual(self.core.epoch, 3)
        self.assertEqual(self.core.holder, 'agent1')

    # --- state 스냅샷 ---------------------------------------------
    def test_state_snapshot_shape(self):
        self.core.acquire('agent1', now=0.0)
        self.core.acquire('agent2', now=0.0)
        snap = self.core.state(now=5.0)
        self.assertEqual(snap['holder'], 'agent1')
        self.assertEqual(snap['queue'], ['agent2'])
        self.assertEqual(snap['epoch'], 1)
        self.assertEqual(snap['lease_remaining_sec'], 25.0)

    def test_state_snapshot_without_now_has_null_remaining(self):
        self.core.acquire('agent1', now=0.0)
        snap = self.core.state()
        self.assertIsNone(snap['lease_remaining_sec'])

    def test_invalid_lease_sec_rejected(self):
        with self.assertRaises(ValueError):
            ZoneTokenCore(lease_sec=0.0)


if __name__ == '__main__':
    unittest.main()
