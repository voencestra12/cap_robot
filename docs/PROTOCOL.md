<!-- 역할: 노드 간 공개 프로토콜. 인터페이스: ROS 2 토픽/서비스 및 JSON schema=1. -->
# ROS 2 프로토콜

ROS 2 Humble의 `std_msgs/msg/String`에 버전 있는 JSON을 담는다. 사용자 정의 ROS interface 패키지를
별도 빌드할 필요가 없다. JSON은 최대 128 KiB, 단일 객체, 중복 key/NaN/Inf를 거절한다.
공간은 m/rad이며 SDK 경계에서만 위치 m를 mm로 변환한다. 그리퍼 position은 SDK pulse 0..850이다.

| 토픽 | 주기/방향 | 주요 필드 |
|---|---|---|
| `/cap/mission_input` | 운용자→WS | request_id, goal |
| `/cap/permit` | WS→모든 gate/agent, 5 Hz | session, token, owner, mission_id, task_id, enabled, cooperative |
| `/cap/assignment` | WS→해당 agent, 활성 시 재게시 | session, token, mission_id, task |
| `/cap/cooperative_assignment` | WS→coordinator | 같은 필드, task.type=cooperative, agent=both |
| `/cap/agent_state` | agent1/agent2/both→서로·WS, 5 Hz | agent_id, boot_id, seq, status, token, mission_id, task_id, detail |
| `/cap/mission_state` | WS→모니터, 5 Hz | 현재 임무/완료 목록/상태/각 agent 및 gate snapshot |
| `/agentN/observations` | 독립 perception→agent/WS | agent_id, seq, stamp, frame_id, valid, objects, source, reason |
| `/agentN/command` | agent/coordinator→gate | command_id, token, mission_id, task_id, agent_id, stamp, action |
| `/agentN/command_result` | gate→agent/coordinator | command_id, token, status, reason, started_at, completed_at |
| `/agentN/gate_state` | gate→모두, 10 Hz | boot_id, seq, token, idle, armed, latched, position, rpy, joints, telemetry_fresh |
| `/agentN/cooperative_plan_request` | coordinator→로컬 agent | request_id, 공통 목표, local/peer scene 및 start, base_to_world 행렬 |
| `/agentN/cooperative_plan_response` | 로컬 agent→coordinator | request_id, token, status, plan 또는 reason |
| `/cap/cooperative_go` | coordinator→두 gate | token, group_id, step_index, execute_at, command_ids |
| `/cap/estop`, `/cap/kill` | 운용자/실패한 노드→gate | `std_msgs/Bool` true로 래치 |
| `/cap/metrics` | 노드/운용자→CSV | event_id, source, event, stamp, latency_s, success, detail, 확장 필드 |
| `/agentN/joint_states` | gate→robot_state_publisher | `sensor_msgs/JointState`, agentN_joint1..6 |
| `/tf`, `/tf_static` | calibration/mount/gate/RSP→소비자 | `geometry_msgs/TransformStamped` 기반 |

`gate_state`, `observations`, `mission_state`, `permit` 발행은 reliable/transient local이다.
실행 명령·배정·GO·결과는 reliable/volatile이다. 센서 영상은 SensorDataQoS(best effort)를 구독한다.
정지는 volatile 구독으로 운용자 일회 메시지와 상시 publisher 둘 다 수신하고, 실제 래치는 gate가 보유한다.
초당 상태 게시와 monotonic 수신 감시를 함께 사용하며 DDS QoS만을 watchdog으로 간주하지 않는다.

## 일반 이동 명령

```json
{"schema":1,"command_id":"unique-command","token":"issued-token","mission_id":"mission",
 "task_id":"task","agent_id":"agent1","stamp":1234567890.0,
 "action":{"kind":"move","position":[0.30,0.0,0.35],"rpy":[3.141592653589793,0.0,0.0],"speed":0.03}}
```

이는 형식 예시다. 실제 stamp·임무·토큰은 현재 값과 일치해야 하며, 임의로 게시하면 거절된다.
`move`는 angular_speed/angular_accel을 추가할 수 있다. 관절·relative·Python/SDK 함수 호출은 허용하지 않는다.
`gripper`의 action은 `{"kind":"gripper","position":850,"speed":500}`이다.
agent의 `observe/search/verify`는 고수준 동작이며 하드웨어 명령으로 직접 통과하지 않는다.

한 gate에는 실행 대기열이 없다. 동일 command_id는 성공·실패 후에도 다시 실행되지 않는다.
성공 결과는 세 번 이상의 새로운 controller 샘플에서 위치/자세/정지 또는 그리퍼 위치/접촉을 확인한 후 발행한다.
`started_at`은 SDK 호출 진입 시각으로, 실제 기구부의 운동 시작을 측정한 값이 아니다.

## 공동 운반 prepare / GO

일반 command에 `phase:"prepare"`, `group_id`, `step_index`, `execute_at`을 추가한다.
permit은 `owner:"both", cooperative:true`이며 두 agent의 같은 token heartbeat를 요구한다.
gate는 해당 명령을 받아도 움직이지 않고 `prepared_*` 필드를 gate_state에 게시한다.
coordinator는 두 gate가 정확히 같은 group/step/execute_at과 각 command_id를 준비했을 때만 GO를 보낸다.

예정 시각은 gate 수신 시 .25~3초 뒤여야 한다. GO가 .1초 전까지 도착하지 않거나
실제 SDK 호출 진입이 기본 .05초 이상 늦으면 정지한다. 각 단계의 양쪽 결과와 최신 idle 상태를
확인하기 전 다음 단계 명령을 보내지 않는다. 서로 다른 토픽의 결과/상태 도착 순서도 처리한다.

## 서비스와 복구

- `/agentN/arm`: `std_srvs/Trigger`. 실제 교정 플래그·controller 상태·펜스 확인 후 활성화.
- `/agentN/reset_stop`: `std_srvs/Trigger`. 실행 중 불가, Kill 해제 불가, 리셋 후에도 unarmed.

임무·worker·안전 gate의 재시작 시 boot_id/session이 바뀐다. 활성 임무에서 재시작을 감지하면
결과 불명확 상태로 중지한다. BLOCKED 임무를 새로운 토큰으로 자동 재할당하지 않는다.
안전 제한은 ROS 런타임 parameter 변경으로 갱신되지 않으며 시작 시 읽은 프로필을 사용한다.
