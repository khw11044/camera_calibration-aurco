# LeKiwi 카메라 캘리브레이션 · ArUco

LeKiwi 로봇 카메라(전방 · 손목)로 **ArUco 마커의 ID · 거리 · 자세를 재기 위한** PC용 도구 모음이다.
순서는 **① 마커 인쇄 → ② 카메라 캘리브레이션 → ③ ArUco 인식 확인**이다.

- 대상: LeKiwi (Raspberry Pi 4 · ROS 2 Jazzy 브링업) + PC (Ubuntu 24.04)
- 영상은 로봇 브링업의 `lekiwi_zmq_host`가 여는 ZMQ 관측 스트림(포트 5556)으로 받는다. **PC에 ROS가 필요 없다**

```text
① aruco_maker/make_aruco_a4_20mm.py ──▶ 마커 PDF (A4, 100% 인쇄)

② 로봇 카메라 ─(ZMQ 5556)─▶ camera_calibration/calibrate_camera.py ──▶ calib/<robot>/<front|wrist>/*.yaml
                                                                            │ (K · 왜곡 계수)
③ 로봇 카메라 ─(ZMQ 5556)─▶ aruco_marker/aruco_viewer.py ◀──────────────────┘
                              마커 테두리 · ID  (+ --robot 주면 좌표축 · 거리)
```

---

## 폴더 구조

```text
camera_calibration-aurco/
├── README.md                          # 이 문서 (전체 개요)
├── camera_calibration/                # ② 체커보드로 카메라 내부 파라미터 구하기
│   ├── README.md                      #    자세한 사용법 · 잘 찍는 법 · 문제 해결
│   ├── calibrate_camera.py
│   ├── test_calibrate_camera.py
│   ├── requirements.txt               #    두 도구 공용 pip 패키지
│   └── calib/<robot>/<front|wrist>/   #    결과 (camera_info YAML + 찍은 JPEG)
└── aruco_marker/
    ├── aruco_maker/
    │   └── make_aruco_a4_20mm.py      # ① 수업용 마커 PDF 만들기
    ├── aruco_viewer.py                # ③ 로봇 영상에서 마커 인식 확인
    └── test_aruco_viewer.py
```

`aruco_viewer.py`는 `../camera_calibration/`의 영상 수신 코드와 `calib/` 결과를 가져다 쓴다. **두 폴더를 같은 위치에 둔 채로** 옮긴다.

---

## 설치 (한 번만)

```bash
conda create -y -n lekiwi_camera python=3.12
conda activate lekiwi_camera
cd ~/workspace/camera_calibration-aurco
pip install -r camera_calibration/requirements.txt
pip install pillow          # ① 마커 PDF 만들 때만 필요
```

설치를 확인한다. 둘 다 `ok`가 나와야 한다 (카메라 · 로봇 없이 돈다).

```bash
(cd camera_calibration && python test_calibrate_camera.py | tail -1)
python aruco_marker/test_aruco_viewer.py
```

> Miniconda 설치, 시스템 Python(apt)으로 돌리는 법은 [camera_calibration/README.md](camera_calibration/README.md#준비)에 있다.
> ROS를 source한 셸이나 lerobot 가상환경에서는 다른 OpenCV가 먼저 잡힐 수 있으니 이 환경을 따로 쓴다.

**로봇**: 웹 오퍼레이터에서 **브링업을 시작한다 (카메라 켬)**. 모터 상태가 없으면 영상도 오지 않으니 모터까지 정상이어야 한다.

---

## ① 마커 인쇄

```bash
cd aruco_marker/aruco_maker
python make_aruco_a4_20mm.py     # → aruco_id0-4_x30_20mm_A4.pdf
```

- A4 5쪽: 한 쪽에 한 ID 씩 30장 (1쪽 = ID 0, 2쪽 = ID 1, … 5쪽 = ID 4). 사전 `DICT_4X4_50`, 검은 테두리 바깥 한 변 **20 mm**
- **실제 크기(100%)로 인쇄한다.** "페이지에 맞춤"이면 마커가 작아져 거리가 틀린다. 인쇄 후 페이지 아래 **100 mm 막대**를 자로 잰다
- 마커 사이를 가운데로 자르면 둘레에 흰 여백이 남는다. 흰 여백이 있어야 잘 잡힌다

## ② 카메라 캘리브레이션

체커보드 [Checkerboard-A4-25mm-10x7.pdf](https://markhedleyjones.com/media/projects/calibration-checkerboard-collection/Checkerboard-A4-25mm-10x7.pdf)를 100%로 인쇄해 평평한 판에 붙인다.

```bash
cd camera_calibration
python calibrate_camera.py --robot lekiwi12 --remote 10.42.0.195
```

카메라(front / wrist)를 고르고, 화면에 `board OK`가 뜰 때 `SPACE`로 20장을 모으면 계산 · 저장한다.
RMS 재투영 오차가 **0.5 px 이하면 좋고, 1.0 px를 넘으면 다시 찍는다.**

결과는 `calib/<robot>/<front|wrist>/<front|wrist>_camera.yaml`(ROS `camera_info` 형식)이다.
끝날 때 출력되는 `scp` 명령으로 로봇의 `~/.ros/camera_info/`에도 넣는다.

옵션, 잘 찍는 법, 문제 해결은 → **[camera_calibration/README.md](camera_calibration/README.md)**

## ③ ArUco 인식 확인

```bash
cd aruco_marker
python aruco_viewer.py --remote 10.42.0.195                    # 테두리 · ID 만
python aruco_viewer.py --remote 10.42.0.195 --robot lekiwi12   # + 좌표축 · 카메라→마커 거리
python aruco_viewer.py --remote 10.42.0.195 --dict auto        # 사전을 모르는 마커
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--remote` | (필수) | 로봇 IP. 유선 직결 `10.42.0.x`가 덜 끊긴다 |
| `--port` | `5556` | 관측 스트림 포트 |
| `--robot` | 없음 | 로봇 이름. 주면 `camera_calibration/calib/<robot>/<camera>/`의 결과로 거리 · 자세를 그린다 |
| `--marker-mm` | `20` | 마커 한 변 [mm] (검은 테두리 바깥, 흰 여백 제외) |
| `--dict` | `4X4_50` | 사전 (예: `6X6_250`, `APRILTAG_36h11`). `auto` = 4X4~7X7 · ARUCO_ORIGINAL · AprilTag 9개 계열을 모두 훑는다 |

키: `q` 또는 `ESC` = 끝내기 (한글 입력 상태면 `ESC`). 창은 영상의 1.5배로 열리고 드래그로 크기를 바꿀 수 있다.

- 거리는 **캘리브레이션한 그 카메라 · 그 해상도**에서만 맞는다. 로봇의 `lekiwi_camera.yaml`에서 해상도 · 회전을 바꾸면 ②부터 다시 한다
- `--marker-mm`이 실제 인쇄 크기와 다르면 거리가 같은 비율로 틀린다

---

## 주의

- **같은 로봇 스트림에 다른 ZMQ 클라이언트(teleop, YOLO, 다른 도구)를 함께 붙이지 않는다.** 프레임을 번갈아 나눠 가진다. 캘리브레이션과 뷰어도 동시에 켜지 않는다
- 캘리브레이션 값은 **로봇마다 따로**다. 같은 기종 카메라라도, SD 카드를 복제한 로봇이라도 다시 찍는다
- 로봇의 `lekiwi_camera` 노드는 아직 이 YAML을 `camera_info` 토픽에 싣지 않는다 (지금은 `K`가 모두 0). 지금은 PC 도구가 `calib/` 파일을 직접 읽는다
