import math
from typing import List, Sequence, Tuple

import cv2
import numpy as np

from bezier_core import Obstacle
from map_loader import load_map_meta, load_pgm_as_binary, pixel_to_world



# 이미 네 코드에 있음:
# @dataclass
# class Obstacle:
#     poly: Optional[Sequence[Point]] = None
#     circle: Optional[Tuple[Point, float]] = None
#     ...

def contours_to_obstacles(mask: np.ndarray,
                          resolution: float,
                          origin: Tuple[float,float,float],
                          approx_eps_m: float = 0.05,
                          min_area_m2: float = 0.02) -> List[Obstacle]:
    """
    - mask: 255=occupied 바이너리
    - approx_eps_m: Douglas-Peucker 근사 허용오차(미터)
    - min_area_m2: 너무 작은 blob 제거
    반환: world 좌표계 polygon 기반 Obstacle 리스트
    """
    H, W = mask.shape
    # 외곽선 추출 (RETR_EXTERNAL: 외곽만)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    obstacles: List[Obstacle] = []
    for cnt in contours:
        if len(cnt) < 3:
            continue
        # 면적 필터링 (픽셀^2 -> m^2)
        area_px = cv2.contourArea(cnt)
        area_m2 = area_px * (resolution ** 2)
        if area_m2 < min_area_m2:
            continue

        # 근사 간소화
        eps_px = max(1.0, approx_eps_m / resolution)  # px 단위
        approx = cv2.approxPolyDP(cnt, epsilon=eps_px, closed=True)  # (N,1,2)

        # 픽셀 -> 월드
        poly_world = []
        for p in approx.reshape(-1, 2):
            u, v = float(p[0]), float(p[1])
            xw, yw = pixel_to_world(u, v, H, resolution, origin)
            poly_world.append((xw, yw))

        # 시계/반시계 상관없음. 필요시 시계방향 통일 가능.
        obstacles.append(Obstacle(poly=poly_world))
    return obstacles

def inflate_mask(mask: np.ndarray, robot_radius_m: float, resolution: float) -> np.ndarray:
    if robot_radius_m <= 1e-9:
        return mask
    r_px = int(math.ceil(robot_radius_m / resolution))
    if r_px <= 0:
        return mask
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*r_px+1, 2*r_px+1))
    inflated = cv2.dilate(mask, kern)
    return inflated


def obstacles_from_ros_map(yaml_path: str,
                           robot_radius_m: float = 0.0,
                           approx_eps_m: float = 0.05,
                           min_area_m2: float = 0.02):
    meta = load_map_meta(yaml_path)
    mask = load_pgm_as_binary(meta['image'], meta['negate'], meta['occ_th'])
    if robot_radius_m > 0:
        mask = inflate_mask(mask, robot_radius_m, meta['resolution'])
    obs = contours_to_obstacles(mask,
                                resolution=meta['resolution'],
                                origin=meta['origin'],
                                approx_eps_m=approx_eps_m,
                                min_area_m2=min_area_m2)
    return obs, meta, mask
