import yaml, cv2, math, os
import numpy as np
from typing import Tuple, List, Sequence

# --- ROS Map YAML 파싱 ---
def load_map_meta(yaml_path: str):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    # 필수 필드
    image = meta['image']               # .pgm 경로 (yaml 기준 상대경로일 수 있음)
    resolution = float(meta['resolution'])  # m/pixel
    origin = tuple(meta['origin'])          # [x, y, yaw]
    negate = int(meta.get('negate', 0))
    occ_th = float(meta.get('occupied_thresh', 0.65))
    free_th = float(meta.get('free_thresh', 0.196))
    # image 경로 보정
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    return {
        'image': image, 'resolution': resolution, 'origin': origin,
        'negate': negate, 'occ_th': occ_th, 'free_th': free_th
    }

# --- PGM 로드(+이진화) ---
def load_pgm_as_binary(pgm_path: str, negate: int, occ_th: float) -> np.ndarray:
    """
    반환: mask (H,W) uint8, 255=occupied, 0=free
    ROS 관례:
      - PGM 0(검정)=점유, 255(흰)=자유 가 기본.   (negate=0)
      - negate=1이면 반대.
      - YAML의 occupied_thresh는 [0..1] 그레이 정규화 임계.
    """
    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)  # (H,W) 8bit
    if img is None:
        raise FileNotFoundError(pgm_path)
    # 정규화: 0..255 -> 0..1
    norm = img.astype(np.float32) / 255.0
    if negate == 0:
        occ = norm <= (1.0 - occ_th)   # 검정(0)에 가까울수록 점유
    else:
        occ = norm >= occ_th           # 흰색(1)에 가까울수록 점유
    mask = np.zeros_like(img, dtype=np.uint8)
    mask[occ] = 255
    return mask  # 255=occupied

# --- 픽셀 <-> 월드 좌표 변환 ---
def pixel_to_world(u: float, v: float, H: int, resolution: float, origin: Tuple[float,float,float]) -> Tuple[float,float]:
    """
    PGM 이미지 기준:
      - (u,v): u는 x-픽셀(→ 오른쪽), v는 y-픽셀(→ 아래로 증가)
      - 월드좌표계: origin = [x0,y0,yaw], 해상도 res(m/px).
      - 맵의 (0,0) 픽셀은 월드 (x0, y0 + H*res) 근처가 아니라, 
        통상적으로 'pixel (0,0)'이 맵 상단-좌측이므로 y축을 뒤집어줘야 함.
    월드 변환식(셀 중심 정렬):
      x = x0 + (u + 0.5) * res
      y = y0 + (H - (v + 0.5)) * res
    yaw(회전)은 대부분 0이므로 무시. (필요하면 회전 행렬 적용 가능)
    """
    x0, y0, yaw = origin
    x = x0 + (u + 0.5) * resolution
    y = y0 + (H - (v + 0.5)) * resolution
    if abs(yaw) > 1e-12:
        c, s = math.cos(yaw), math.sin(yaw)
        # 원점에서의 회전: 먼저 원점 보정 후 회전하려면 추가 처리 필요
        # 대부분 yaw=0이므로 생략. 필요시 아래처럼 상대좌표 회전 적용 가능.
        # rx = x0 + c*(x-x0) - s*(y-y0)
        # ry = y0 + s*(x-x0) + c*(y-y0)
        # return (rx, ry)
    return (x, y)
