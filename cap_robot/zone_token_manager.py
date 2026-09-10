"""A+B 공용 구역 토큰 매니저 ROS2 노드.

정확히 1개 인스턴스만 실행합니다. 두 Agent(robot_agent)는 다음 토픽으로 통신합니다.

- 요청:  {request_topic}  std_msgs/String, JSON
    {"agent_id": "agent1", "action": "acquire"|"release"|"abandon",
     "epoch": <int, release 시 권장>, "request_id": "<옵션, 에코>"}

- 상태:  {state_topic}  std_msgs/String, JSON (latched / TRANSIENT_LOCAL)
    {"holder": "agent1"|null, "queue": ["agent2"], "epoch": 7,
     "lease_remaining_sec": 42.1, "last_event": "...", "stamp": <float>}

안전 로직은 순수 파이썬 ZoneTokenCore 가 담당하고, 이 노드는 I/O만 담당합니다.
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from .zone_token import (
        ABANDON,
        ACQUIRE,
        GRANTED,
        RELEASE,
        RELEASED,
        ZoneTokenCore,
    )
except ImportError:  # 소스 디렉터리에서 직접 실행할 때
    from zone_token import (
        ABANDON,
        ACQUIRE,
        GRANTED,
        RELEASE,
        RELEASED,
        ZoneTokenCore,
    )


class ZoneTokenManager(Node):
    def __init__(self):
        super().__init__('zone_token_manager')

        self.declare_parameter('request_topic', '/zone_token/request')
        self.declare_parameter('state_topic', '/zone_token/state')
        self.declare_parameter('lease_sec', 45.0)
        self.declare_parameter('tick_period_sec', 1.0)

        self.request_topic = str(self.get_parameter('request_topic').value).strip()
        self.state_topic = str(self.get_parameter('state_topic').value).strip()
        self.lease_sec = float(self.get_parameter('lease_sec').value)
        tick_period_sec = max(0.1, float(self.get_parameter('tick_period_sec').value))

        self.declare_parameter('status_log_period_sec', 10.0)
        self.status_log_period_sec = max(
            1.0, float(self.get_parameter('status_log_period_sec').value)
        )

        self.core = ZoneTokenCore(lease_sec=self.lease_sec)
        self._last_event = 'STARTED'
        self._last_status_log = 0.0

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(
            String, self.state_topic, latched_qos
        )
        self.request_sub = self.create_subscription(
            String, self.request_topic, self._on_request, 10
        )
        self.tick_timer = self.create_timer(tick_period_sec, self._on_tick)

        self.get_logger().info(
            f'🎟️ zone_token_manager 시작: lease={self.lease_sec:.1f}s, '
            f'request={self.request_topic}, state={self.state_topic}'
        )
        self._publish_state()

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return time.monotonic()

    def _on_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('요청은 JSON 객체여야 합니다.')
            agent_id = str(payload.get('agent_id', '')).strip()
            action = str(payload.get('action', '')).strip().lower()
            if not agent_id:
                raise ValueError('agent_id가 비어 있습니다.')
        except (ValueError, json.JSONDecodeError) as error:
            self.get_logger().warn(f'⚠️ 잘못된 토큰 요청 무시: {error}')
            return

        now = self._now()

        if action == ACQUIRE:
            result, epoch = self.core.acquire(agent_id, now)
            self._last_event = f'{agent_id} acquire -> {result}'
            if result == GRANTED:
                self.get_logger().info(
                    f'✅ [{agent_id}] 공용 구역 토큰 획득 (epoch={epoch})'
                )
            else:
                self.get_logger().info(
                    f'⏳ [{agent_id}] 공용 구역 대기열 진입 '
                    f'(holder={self.core.holder}, queue={self.core.waiters})'
                )
        elif action == RELEASE:
            raw_epoch = payload.get('epoch')
            try:
                epoch = None if raw_epoch is None else int(raw_epoch)
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f'⚠️ [{agent_id}] release epoch 형식 오류({raw_epoch!r}) → epoch 없이 처리'
                )
                epoch = None
            result, reason = self.core.release(agent_id, epoch, now)
            self._last_event = f'{agent_id} release -> {result}'
            if result == RELEASED:
                self.get_logger().info(
                    f'🟢 [{agent_id}] 공용 구역 토큰 반납 '
                    f'-> holder={self.core.holder}, queue={self.core.waiters}'
                )
            else:
                self.get_logger().warn(
                    f'🚫 [{agent_id}] 반납 무시({reason}) '
                    f'(현재 holder={self.core.holder}, epoch={self.core.epoch})'
                )
        elif action == ABANDON:
            changed = self.core.abandon(agent_id, now)
            self._last_event = f'{agent_id} abandon -> {"changed" if changed else "noop"}'
            if changed:
                self.get_logger().warn(
                    f'🧹 [{agent_id}] liveness 상실 처리 '
                    f'-> holder={self.core.holder}, queue={self.core.waiters}'
                )
        else:
            self.get_logger().warn(f'⚠️ 알 수 없는 action: {action!r}')
            return

        self._publish_state()

    def _on_tick(self) -> None:
        now = self._now()
        changed, reclaimed = self.core.tick(now)
        if changed:
            self._last_event = f'lease expired -> reclaimed {reclaimed}'
            self.get_logger().warn(
                f'⏰ [{reclaimed}] lease 만료로 토큰 강제 회수 '
                f'-> holder={self.core.holder}, queue={self.core.waiters}'
            )
            self._publish_state()

        # P5: 토큰이 점유 중일 때 주기적으로 상태를 로그로 남겨 '멈춘 토큰'을 추적.
        if self.core.holder is not None and (
            now - self._last_status_log >= self.status_log_period_sec
        ):
            self._last_status_log = now
            snap = self.core.state(now)
            self.get_logger().info(
                f"🎟️ 보유중: holder={snap['holder']}, queue={snap['queue']}, "
                f"epoch={snap['epoch']}, lease_remaining={snap['lease_remaining_sec']}s"
            )

    def _publish_state(self) -> None:
        snapshot = self.core.state(self._now())
        snapshot['last_event'] = self._last_event
        snapshot['stamp'] = self._now()
        msg = String()
        msg.data = json.dumps(snapshot, ensure_ascii=False)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZoneTokenManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
