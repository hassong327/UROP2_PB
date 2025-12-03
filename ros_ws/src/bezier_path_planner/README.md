# ROS2 패키지: bezier_path_planner

공통 `bezier_core`를 이용해 Nav2 costmap을 피하는 베지어 경로를 만들어 `nav_msgs/Path`로 발행하는 ROS2 ament_python 패키지입니다. `/clicked_point`로 받은 start/goal을 연결합니다.

## 빌드 & 실행 (`ros_ws` 기준)
```bash
cd <repo-root>
pip install -e common                 # 코어 설치 (한 번만)
cd ros_ws
colcon build --packages-select bezier_path_planner
source install/setup.bash
ros2 launch bezier_path_planner bezier_planner.launch.py
```
RViz에서 두 번 클릭해 `/clicked_point`를 찍으면, `planner/path` 토픽에 경로가 발행됩니다.

## 핵심 파일
- `bezier_path_planner/utils.py`        : 코어 로직 import + costmap 샘플/혼합 충돌 체크
- `bezier_path_planner/bezier_planner_node.py` : start/goal→경로 생성 후 Path 발행
- `launch/bezier_planner.launch.py`     : 실행용 launch
