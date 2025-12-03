// ...existing code...
# 베지어 경로 계획 (monorepo)

공통 코어, 가벼운 데모, ROS2 패키지를 포함한 모노레포 구조:

```
root/
├── common/             # pip으로 설치 가능한 공유 코어 (bezier_core)
├── demo/               # 독립 실행형 데모 (PGM/YAML 맵)
└── ros_ws/
    └── src/
        └── bezier_path_planner/  # ROS2 ament_python 패키지
```

## 설정
1) 공통 코어를 한 번 설치 (데모와 ROS2에서 공통 사용)
```bash
pip install -e common
```

2) 데모 실행 (리포지토리 루트에서)
```bash
python demo/main.py
```

3) ROS2 패키지 빌드 (ros_ws 내부)
```bash
cd ros_ws
colcon build --packages-select bezier_path_planner
source install/setup.bash
ros2 launch bezier_path_planner bezier_planner.launch.py
```

## 참고
- `bezier_core`가 기하/QP 로직의 단일 소스입니다.
- 데모와 ROS2 코드는 코어를 임포트하므로, 실행 전에 `pip install -e common`을 수행하세요.# UROP2_PB
