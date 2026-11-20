# lekiwi_camera_calibration

LeKiwi **전방 · 손목 카메라의 내부 파라미터(K · 왜곡 계수)를 PC에서 체커보드로 구하고, 로봇에 ROS `camera_info` YAML로 넣는** 도구.
ArUco 마커로 거리 · 자세를 재려면 이 값이 먼저 있어야 한다.

- 대상: LeKiwi (Raspberry Pi 4 · ROS 2 Jazzy 브링업) + PC (Ubuntu 24.04)
- 영상 경로: 로봇 브링업의 `lekiwi_zmq_host`가 여는 ZMQ 관측 스트림 (포트 5556). `YOLO/01.take_video.py`와 같은 방식이라 **PC에 ROS가 필요 없다**

```text
로봇 카메라 ─ lekiwi_camera 노드 ─ lekiwi_zmq_host ─(ZMQ 5556, JPEG 원본)─▶ PC calibrate_camera.py
                                                                            │ SPACE x20
                                                                            ▼
                    로봇 ~/.ros/camera_info/<front|wrist>_camera.yaml ◀─scp─ calib/<robot>/<front|wrist>/*.yaml
```

---

## 준비

**체커보드**: [Checkerboard-A4-25mm-10x7.pdf](https://markhedleyjones.com/media/projects/calibration-checkerboard-collection/Checkerboard-A4-25mm-10x7.pdf)

- A4에 **100% (실제 크기)로** 인쇄한다. "용지에 맞춤"으로 인쇄하면 칸 크기가 바뀐다
- 휘지 않게 평평한 판에 붙인다
- 내부 코너 10 × 7 (칸 11 × 8), 칸 25 mm. 스크립트 기본값이 이 보드다. 다른 보드는 `--cols` `--rows` `--square-mm`로 바꾼다 (내부 코너 = 칸 수 − 1)

**PC — conda 가상환경** (한 번만)

이 환경 하나로 캘리브레이션(`calibrate_camera.py`)과 ArUco 확인(`../aruco_marker/aruco_viewer.py`)을 둘 다 돌린다.

1. Miniconda가 없으면 설치한다 (있으면 2번으로).
   ```bash
   wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
   bash Miniconda3-latest-Linux-x86_64.sh      # 안내대로 yes, 설치 경로는 기본값
   source ~/.bashrc                            # 또는 터미널을 새로 연다
   ```
2. 가상환경을 만들고 패키지를 설치한다.
   ```bash
   conda create -y -n lekiwi_camera python=3.12
   conda activate lekiwi_camera
   cd ~/workspace/camera_calibration-aurco/camera_calibration
   pip install -r requirements.txt
   ```
3. 설치를 확인한다. 둘 다 `ok`가 나와야 한다 (카메라 · 로봇 없이 돈다).
   ```bash
   python test_calibrate_camera.py | tail -1
   python ../aruco_marker/test_aruco_viewer.py
   ```

이후에는 터미널을 열 때마다 `conda activate lekiwi_camera`를 먼저 하고, 아래 명령의 `python3`는 `python`으로 쳐도 된다.
끝낼 때는 `conda deactivate`.

> conda 없이 시스템 Python으로 돌리려면 `sudo apt install python3-opencv python3-yaml python3-zmq` (OpenCV 4.6). 두 방식 모두 돈다.
> ROS 를 source 한 셸이나 lerobot 가상환경 안에서는 다른 OpenCV 가 먼저 잡힐 수 있으니 이 환경을 따로 쓴다.

**로봇**: 웹 오퍼레이터에서 **브링업을 시작한다 (카메라 켬)**. 브링업이 `lekiwi_zmq_host`로 5556 포트를 연다.
`lekiwi_zmq_host`는 팔 · 바퀴 상태가 없으면 영상도 보내지 않으므로 모터까지 정상이어야 한다.

---

## 실행

```bash
cd ~/workspace/camera_calibration-aurco/camera_calibration
python3 calibrate_camera.py --robot lekiwi12 --remote 10.42.0.195
```

시작하면 터미널에서 카메라를 고른다. 번호, 이름, 엔터(=front) 모두 된다.

```text
[스트림에 실린 카메라]
  1) front   <- 기본값
  2) wrist
캘리브레이션할 카메라 (번호 또는 이름, 엔터=front):
```

영상 창이 열리면 보드를 비추고, 왼쪽 위가 `board OK`가 되고 코너에 색 점이 찍힐 때 SPACE를 누른다. 20장을 채우면 바로 계산해 저장한다.

| 키 | 동작 |
|----|------|
| `SPACE` | 지금 화면을 한 장으로 넣는다 (보드가 잡혔을 때만) |
| `r` | 모은 장을 버리고 처음부터 |
| `q` / `ESC` | 저장하지 않고 끝낸다 (한글 입력 상태면 `q`가 안 먹으니 `ESC`) |

창은 영상의 1.5배로 열리고, 드래그로 크기를 바꿀 수 있다.

**옵션**

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--robot` | (필수) | 로봇 이름. 결과 폴더 이름이 된다 |
| `--remote` | (필수) | 로봇 IP. 유선 직결이면 `10.42.0.x`가 영상이 덜 끊긴다 (로봇에서 `hostname -I`) |
| `--port` | `5556` | 관측 스트림 포트 |
| `--cols` / `--rows` | `10` / `7` | 보드 내부 코너 수 |
| `--square-mm` | `25` | 칸 한 변 [mm] |
| `--shots` | `20` | 모을 장수 (최소 10) |

---

## 잘 찍는 법

- **로봇은 고정하고 보드를 움직인다.** 손목 카메라는 찍는 동안 팔을 움직이지 않는다
- **보드 전체와 바깥 흰 테두리가 화면 안에** 있어야 잡힌다. 한쪽이 잘리거나 물건에 가리면 안 잡힌다
- 화면 가운데만 찍지 말고 **네 귀퉁이 · 가장자리**까지 보드를 보낸다. 렌즈 왜곡은 가장자리에서 드러난다
- 전방 카메라는 바닥 가까이 달려 있어 **화면 아래쪽이 비기 쉽다.** 보드를 바닥 가까이 낮게 들고 위로 젖혀 화면 아래 끝까지 오게 한 장을 꼭 넣는다
- 보드를 상하좌우로 **30~45° 기울인** 장과, **가깝고 먼** 장을 섞는다
- 보드가 **멈춘 뒤** SPACE를 누른다 (흔들리면 코너가 번진다)

끝나면 이렇게 출력된다.

```text
RMS 재투영 오차: 0.312 px  (0.5 이하 좋음, 1.0 넘으면 다시 찍기)
오차 큰 장: 07.jpg 0.61px, 13.jpg 0.48px, 02.jpg 0.40px
```

RMS가 1.0 px를 넘으면 다시 찍는다. 오차 큰 장의 사진을 열어 흔들렸거나 보드가 휘었는지 본다.

---

## 결과

카메라마다 폴더가 하나다. 다시 찍으면 그 폴더를 덮어쓴다 (지난번 JPEG 는 지우고 새로 쓴다).

```text
calib/<robot>/<front|wrist>/
├── front_camera.yaml   # ROS camera_info 형식 (camera_calibration_parsers)
├── 00.jpg … 19.jpg     # 찍은 원본 JPEG (카메라가 보낸 그대로)
```

```yaml
# lekiwi12 front_camera 20261120_120000: RMS 0.312 px, 20 shots, board 10x7 inner corners @ 25 mm, via lekiwi_zmq_host
image_width: 640
image_height: 480
camera_name: front_camera
camera_matrix: {rows: 3, cols: 3, data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]}
distortion_model: plumb_bob
distortion_coefficients: {rows: 1, cols: 5, data: [k1, k2, p1, p2, k3]}
rectification_matrix: …      # 단위 행렬 (단안 카메라)
projection_matrix: …         # ROS cameracalibrator 와 같게 alpha=0
```

첫 줄 주석에 로봇 · 시각 · RMS · 보드 · 어느 호스트로 찍었는지가 남는다.

---

## 로봇에 넣기

끝날 때 실제 경로(절대 경로)로 채운 명령이 출력된다. 그대로 **PC에서** 실행한다 (로봇에 들어갈 필요 없다).
직접 칠 때는 `calib/...` 가 이 폴더 기준이므로 먼저 이 폴더로 이동한다.

```bash
cd ~/workspace/camera_calibration-aurco/camera_calibration
ssh roboseasy@10.42.0.195 'mkdir -p ~/.ros/camera_info'
scp calib/lekiwi12/front/front_camera.yaml roboseasy@10.42.0.195:.ros/camera_info/front_camera.yaml
```

**왜 로봇의 `~/.ros/camera_info/`인가**

- 렌즈 왜곡은 카메라마다 다르다. 같은 기종이라도 **로봇마다 따로 찍어야 하는 로봇별 데이터**다.
  그래서 모든 로봇이 함께 쓰는 레포의 `lekiwi_camera.yaml`에는 넣지 않는다
- ROS `camera_info`의 관례 위치이고, `git pull` · 다시 빌드해도 지워지지 않는다

---

## 주의

- **해상도 · 회전에 묶인다.** 로봇의 `lekiwi_camera.yaml`에서 `width` · `height` · `rotation`을 바꾸면 다시 찍는다
- **ArUco가 쓸 영상과 같은 경로로 찍는다.** lerobot `lekiwi_host`에도 붙지만, 그 영상은 lerobot 설정의 해상도 · 회전(전방 180°)을 따른다.
  스크립트가 시작할 때와 YAML 첫 줄에 어느 호스트였는지 남긴다
- **같은 로봇 스트림에 다른 ZMQ 클라이언트(teleop, YOLO 등)를 함께 붙이지 않는다.** 프레임을 번갈아 나눠 가진다
- **고른 이름과 화면이 다르면** (front를 골랐는데 그리퍼가 보이면) 그 로봇은 카메라 장치가 뒤바뀌어 있다. 로봇의 `lekiwi_camera.yaml` `device`부터 고친다
- **SD 카드 이미지를 복제하면 `~/.ros/camera_info`도 따라간다.** 복제한 로봇은 다시 찍는다

---

## 문제 해결

| 증상 | 원인 · 조치 |
|------|------------|
| `pyzmq 가 필요하다` | conda 환경을 켜지 않았음 (`conda activate lekiwi_camera`). 시스템 Python 이면 `sudo apt install python3-zmq` |
| `10 초 안에 관측이 오지 않았다` | 브링업이 꺼져 있음 · 모터 상태 없음 · IP가 틀림 · 다른 클라이언트가 붙어 있음 |
| `스트림에 카메라가 없다` | 브링업에서 카메라를 켜지 않음 |
| 화면에 계속 `board --` | 보드가 잘리거나 가려짐 · `--cols/--rows`가 보드와 다름 (내부 코너 = 칸 수 − 1) · 너무 멀거나 조명 반사 |
| `영상이 끊겼다` | Wi-Fi 대역폭 부족. 유선 직결 IP(`10.42.0.x`)로 붙는다. 모은 장은 유지되고, 끊긴 동안은 찍히지 않는다 |
| `q`가 안 먹음 | 한글 입력 상태. `ESC`를 누른다 |
| RMS 1.0 px 초과 | 흔들린 장 · 휜 보드 · 한쪽에 몰린 장. 위 "잘 찍는 법"대로 다시 |

---

## 자체 점검

카메라 · 로봇 · pyzmq 없이 돈다.

```bash
python3 test_calibrate_camera.py      # ok
```

- 값을 아는 가상 카메라로 20장을 만들어 K · 왜곡을 되찾는지
- 저장한 YAML을 다시 읽으면 같은 값인지
- 관측 JSON의 영상을 호스트 종류(ROS / lerobot)에 맞게 푸는지
- 카메라 고르기 입력(번호 · 이름 · 엔터 · 잘못된 값)

---

## 파일 구조

```text
camera_calibration/
├── README.md
├── calibrate_camera.py         # 실행 스크립트
├── test_calibrate_camera.py    # 자체 점검
├── requirements.txt            # pip 패키지 (conda 환경용, aruco_marker 와 공용)
└── calib/<robot>/<front|wrist>/ # 결과 (카메라마다 하나, 다시 찍으면 덮어씀)
```

## 알려진 한계

- **로봇의 `lekiwi_camera` 노드가 아직 이 YAML을 읽지 않는다.** 지금 발행하는 `camera_info`는 `K`가 모두 0이다.
  다음 단계에서 노드가 `~/.ros/camera_info/<이름>.yaml`을 읽어 `camera_info`에 싣게 하면, ArUco 코드는 토픽에서 값을 받는다
- 카메라 노출 · 초점은 카메라 자동 설정 그대로다. 오토포커스 카메라라면 초점이 바뀔 때 값도 바뀐다

## 참고한 코드

- `Cal_checkerboard_yolo/01.calibrate.py`: SPACE 촬영 루프, `cornerSubPix`, `calibrateCamera`
- `YOLO/01.take_video.py` · `frame_source.py`: `--remote` ZMQ 수신, 카메라 고르기, 크기 조절 창, lerobot 호스트 색 되돌리기
- ROS `camera_calibration_parsers` YAML 형식, ROS `cameracalibrator`의 projection 행렬 (alpha=0)

## 검증 기록

- 2026-11-20, PC: 자체 점검 4개 통과. 가짜 ZMQ 호스트로 끝까지 실행 (중복 없이 저장, 3초 끊김 뒤 이어 찍기, wrist 선택 시 `wrist_camera.yaml`), 창 크기 960×720 확인
- 2026-11-20, lekiwi12: 실제 스트림 수신 확인 (`lekiwi_zmq_host`, 640×480, front = 정면, wrist = 그리퍼, 색 정상).
  화면에 잡힌 Checkerboard-A4-25mm-10x7을 10×7로 검출 확인
- 2026-11-20, lekiwi12 전방 첫 실행 (`calib/lekiwi12/20261120_123413_front`): RMS 0.468 px, fx 620.7 · fy 620.2, cx 316.6 · cy 270.9.
  다만 코너가 화면 아래 1/4(y > 346)에 한 장도 없고 왼쪽이 적어, 왜곡 계수(k2 0.247, k3 −0.664)는 그 영역을 짐작으로 채운 값이다 → 아래 · 왼쪽을 채워 다시 찍을 것.
  칸 길이 기본값을 25 mm로 고치기 전이라 주석에 20 mm로 남았다 (K · 왜곡 값에는 영향 없음)
