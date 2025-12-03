import matplotlib.pyplot as plt
from pathlib import Path

from drawing_map import show_map_and_obstacles
from bezier_core import recursive_push_away
from make_obstacle import obstacles_from_ros_map



if __name__ == "__main__":
    base_dir = Path(__file__).resolve().parent
    yaml_path = base_dir / "figures/map.yaml"
    
    P = [(0.0, 0.0), (1.0, 2.0), (2.0, -2.0), (3.0, 0.0),
         (4.0, 1.0), (5.0, 0.0), (6.0, -1.0), (7.0, 0.0)]

    obstacles, meta, mask = obstacles_from_ros_map(
        yaml_path,
        robot_radius_m=0.20,   # 로봇 반경 20cm 가정 (필요 시 조정)
        approx_eps_m=0.07,     # 컨투어 근사 허용오차
        min_area_m2=0.02       # 작은 잡음 제거
    )

    # 3) 배경 맵 + 초기 장애물 시각화 (옵션)
    fig, ax = plt.subplots(figsize=(7,6))
    show_map_and_obstacles(meta, mask, obstacles, ax=ax, alpha=0.3)
    xs = [p[0] for p in P]; ys = [p[1] for p in P]
    ax.plot(xs, ys, 'b-', lw=2, label='initial curve')
    ax.plot(xs, ys, 'o--', ms=4, label='control poly')
    ax.legend(); plt.show(block=False); plt.pause(5)

    # 4) 충돌 해소
    P_final, remain, info = recursive_push_away(
        P, obstacles,
        flat_eps=5e-3, max_depth=28,
        rho_init=0.05, lambda_smooth=0.5,
        tol_improve=1e-4, rho_growth=1.6, rho_max=1.0,
        max_calls=12, verbose=True,
        plot=True,      # 단계별 곡선 출력
        save_dir=None,  # 저장하려면 "figs"
        samples=800
    )

    print("info:", info)
    print("remain:", remain)
