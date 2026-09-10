"""A+B 공용 구역 점유권(토큰)의 순수 파이썬 코어 로직.

rclpy에 의존하지 않으므로 단위 테스트가 쉽습니다. ROS 노드 래퍼는
zone_token_manager.py 가 담당하며, 이 파일은 다음만 책임집니다.

- 한 번에 한 Agent만 공용 구역 토큰을 보유 (holder)
- 대기자는 FIFO 큐로 관리하고, 반납 시 대기자에게 우선 양보 (공정 큐)
- grant마다 epoch 증가 → lease 회수 후 지각 release 를 무시
- lease_sec 안에 반납/갱신이 없으면 tick() 이 강제 회수
"""

from __future__ import annotations

from typing import List, Optional, Tuple

ACQUIRE = 'acquire'
RELEASE = 'release'
ABANDON = 'abandon'

GRANTED = 'GRANTED'
QUEUED = 'QUEUED'
RELEASED = 'RELEASED'
IGNORED = 'IGNORED'


class ZoneTokenCore:
    """공용 구역 토큰의 소유권/대기열/lease 상태 기계."""

    def __init__(self, lease_sec: float = 45.0):
        if lease_sec <= 0.0:
            raise ValueError('lease_sec은 0보다 커야 합니다.')
        self.lease_sec = float(lease_sec)
        self.holder: Optional[str] = None
        self.waiters: List[str] = []
        self.epoch: int = 0
        self.lease_deadline: Optional[float] = None

    # ------------------------------------------------------------------
    # 공개 API
    # ------------------------------------------------------------------
    def acquire(self, agent_id: str, now: float) -> Tuple[str, Optional[int]]:
        """토큰을 요청합니다. (GRANTED, epoch) 또는 (QUEUED, None) 을 반환합니다.

        - 이미 holder 인 경우: 재진입으로 보고 lease만 갱신, epoch 유지 (하트비트 겸용).
        - 비어 있으면: 즉시 grant.
        - 다른 Agent가 보유 중이면: FIFO 큐에 1회만 추가하고 QUEUED.
        """
        agent_id = str(agent_id)
        if self.holder == agent_id:
            self.lease_deadline = now + self.lease_sec
            return GRANTED, self.epoch
        if self.holder is None:
            self._grant(agent_id, now)
            return GRANTED, self.epoch
        if agent_id not in self.waiters:
            self.waiters.append(agent_id)
        return QUEUED, None

    def release(
        self,
        agent_id: str,
        epoch: Optional[int],
        now: float,
    ) -> Tuple[str, Optional[str]]:
        """토큰을 반납합니다. (RELEASED, None) 또는 (IGNORED, reason) 을 반환합니다.

        holder 가 아니거나 epoch 가 현재 grant 와 다르면 무시합니다.
        epoch 가 None 이면 holder 일치만으로 허용합니다(P2 이전 하위호환).
        """
        agent_id = str(agent_id)
        if self.holder != agent_id:
            return IGNORED, 'not_holder'
        if epoch is not None and int(epoch) != self.epoch:
            return IGNORED, 'stale_epoch'
        self._handoff_or_free(now)
        return RELEASED, None

    def abandon(self, agent_id: str, now: float) -> bool:
        """Agent liveness 상실 시 호출. 대기열에서 제거하고, holder면 회수합니다."""
        agent_id = str(agent_id)
        changed = False
        if agent_id in self.waiters:
            self.waiters = [w for w in self.waiters if w != agent_id]
            changed = True
        if self.holder == agent_id:
            self._handoff_or_free(now)
            changed = True
        return changed

    def tick(self, now: float) -> Tuple[bool, Optional[str]]:
        """lease 만료를 검사합니다. (changed, reclaimed_agent_id) 를 반환합니다."""
        if (
            self.holder is not None
            and self.lease_deadline is not None
            and now >= self.lease_deadline
        ):
            reclaimed = self.holder
            self._handoff_or_free(now)
            return True, reclaimed
        return False, None

    def state(self, now: Optional[float] = None) -> dict:
        """관측용 스냅샷. now 를 주면 lease_remaining_sec 를 함께 계산합니다."""
        remaining = None
        if self.lease_deadline is not None and now is not None:
            remaining = round(self.lease_deadline - now, 3)
        return {
            'holder': self.holder,
            'queue': list(self.waiters),
            'epoch': self.epoch,
            'lease_remaining_sec': remaining,
        }

    # ------------------------------------------------------------------
    # 내부 헬퍼
    # ------------------------------------------------------------------
    def _grant(self, agent_id: str, now: float) -> None:
        self.waiters = [w for w in self.waiters if w != agent_id]
        self.holder = agent_id
        self.epoch += 1
        self.lease_deadline = now + self.lease_sec

    def _handoff_or_free(self, now: float) -> None:
        if self.waiters:
            self._grant(self.waiters.pop(0), now)
        else:
            self.holder = None
            self.lease_deadline = None
