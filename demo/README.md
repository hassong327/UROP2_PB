# 독립형 데모 (PGM 맵)

정적 ROS 맵(`figures/map.yaml`/`.pgm`)을 불러와 컨투어로 다각형 장애물을 만들고, 공통 코어(`bezier_core`)의 QP 기반 push-away로 제어다각형을 밀어내는 예제입니다.

## 실행 방법
레포 루트에서:
```bash
pip install -e common
python demo/main.py
```
실행 시 matplotlib 창에 초기 곡선과 충돌 해소 과정이 순차적으로 나타납니다.

## 주요 파일
- `main.py`          : 데모 엔트리포인트, `recursive_push_away` 호출
- `map_loader.py`    : YAML/PGM 로드, 픽셀↔월드 좌표 변환
- `make_obstacle.py` : 컨투어 추출/팽창 후 `Obstacle` 리스트 생성
- `drawing_map.py`   : 맵+장애물 시각화 유틸
