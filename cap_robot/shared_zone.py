"""A+B 공용 구역 판정에 쓰는 순수 2D 기하 함수 (workspace_0 기준, mm).

rclpy 의존이 없어 단위 테스트가 쉽습니다. robot_agent.py 가 import 해서
"이 모션이 공용 구역에 들어가는가"를 결정론적으로 판정합니다.

box 는 {'x_min','x_max','y_min','y_max'} dict 입니다 (닫힌 구간).
"""

from __future__ import annotations

_EPS = 1e-9


def expand_aabb(box: dict, margin: float) -> dict:
    """AABB를 사방으로 margin(mm) 만큼 확장합니다."""
    m = float(margin)
    return {
        'x_min': float(box['x_min']) - m,
        'x_max': float(box['x_max']) + m,
        'y_min': float(box['y_min']) - m,
        'y_max': float(box['y_max']) + m,
    }


def point_in_aabb(x: float, y: float, box: dict) -> bool:
    """점 (x, y) 가 닫힌 AABB 안(경계 포함)에 있으면 True."""
    return (
        float(box['x_min']) <= float(x) <= float(box['x_max'])
        and float(box['y_min']) <= float(y) <= float(box['y_max'])
    )


def segment_intersects_aabb(
    x0: float, y0: float, x1: float, y1: float, box: dict
) -> bool:
    """닫힌 선분 (x0,y0)-(x1,y1) 이 AABB와 만나면 True (Liang-Barsky).

    끝점이 box 안에 있거나, 선분이 box를 관통/스치는 경우 모두 True 입니다.
    """
    x0, y0, x1, y1 = float(x0), float(y0), float(x1), float(y1)
    if point_in_aabb(x0, y0, box) or point_in_aabb(x1, y1, box):
        return True

    dx = x1 - x0
    dy = y1 - y0
    t0, t1 = 0.0, 1.0

    clips = (
        (-dx, x0 - float(box['x_min'])),
        (dx, float(box['x_max']) - x0),
        (-dy, y0 - float(box['y_min'])),
        (dy, float(box['y_max']) - y0),
    )
    for p, q in clips:
        if abs(p) < _EPS:
            # 이 축으로 평행. 슬래브 밖이면 교차 없음.
            if q < 0.0:
                return False
            continue
        r = q / p
        if p < 0.0:
            if r > t1:
                return False
            if r > t0:
                t0 = r
        else:
            if r < t0:
                return False
            if r < t1:
                t1 = r
    return t0 <= t1


def classify_shared_zone_entry(
    ox: float, oy: float, px: float, py: float, box: dict
):
    """pick/place workspace_0 좌표로 공용 구역 진입을 분류합니다.

    반환: (acquire_before_api, reason)
    - acquire_before_api 는 'move_to_object' | 'move_to_place' | None
    - pick 지점이 구역 안이면 첫 접근(move_to_object) 전에 토큰이 필요
    - place 지점이 구역 안이거나 pick->place 직선이 구역을 통과하면
      첫 배치 이동(move_to_place) 전에 토큰이 필요
    """
    if point_in_aabb(ox, oy, box):
        return 'move_to_object', f'pick ({ox:.0f},{oy:.0f})가 공용 구역 내부'
    if point_in_aabb(px, py, box):
        return 'move_to_place', f'place ({px:.0f},{py:.0f})가 공용 구역 내부'
    if segment_intersects_aabb(ox, oy, px, py, box):
        return 'move_to_place', 'pick→place 경로가 공용 구역을 통과'
    return None, '공용 구역 미접촉'
