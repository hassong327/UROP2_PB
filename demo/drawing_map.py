import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle

def show_map_and_obstacles(meta, mask, obstacles, ax=None, alpha=0.35):
    """
    mask는 (H,W) 바이너리. 월드좌표로 imshow extent 를 맞춰서 배경 표시.
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(7,6))
    H, W = mask.shape
    res = meta['resolution']
    x0, y0, yaw = meta['origin']

    # 월드 범위 (셀 중심기준으로 약간 확장)
    x_min = x0
    x_max = x0 + W * res
    # y축은 위가 +이므로, pixel v=0(상단) 이 y_max에 해당
    y_min = y0
    y_max = y0 + H * res

    # 배경: 점유(255) 부분만 보이게(컬러맵/알파)
    ax.imshow(mask[::-1, :],            # 상하 반전하여 월드 y상향과 맞춤
              extent=[x_min, x_max, y_min, y_max],
              cmap='gray', alpha=alpha, origin='lower')

    # 폴리곤 그리기
    for obs in obstacles:
        if obs.poly is not None:
            ax.add_patch(Polygon(obs.poly, fill=False, linewidth=1.5))
        elif obs.circle is not None:
            (c, r) = obs.circle
            ax.add_patch(Circle(c, r, fill=False, linewidth=1.5))
    ax.set_aspect('equal'); ax.grid(True); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
