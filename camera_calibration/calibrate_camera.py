#!/usr/bin/env python3
"""LeKiwi 카메라(전방 · 손목) 내부 파라미터(K · 왜곡) 캘리브레이션 — PC 에서 실행한다.

로봇의 ZMQ 관측 스트림(포트 5556, lerobot LeKiwi 호스트 형식)에 실린 카메라 중 하나를 골라
(1) front  2) wrist) PC 화면에 띄우고, 체커보드가 잡힌 프레임에서 SPACE 를 누를 때마다 한 장씩 모은다.
목표 장수를 채우면 cv2.calibrateCamera 로 계산해 ROS camera_info YAML(camera_calibration_parsers 형식)로 저장한다.

준비:
	로봇: 웹 오퍼레이터에서 브링업 시작 (카메라 켬). 브링업이 lekiwi_zmq_host 로 5556 을 연다.
		lerobot lekiwi_host 에도 붙지만, 그 영상은 lerobot 설정의 해상도 · 회전을 따른다 —
		ArUco 도 같은 경로로 받을 때만 그 결과가 맞는다.
	PC (한 번만): sudo apt install python3-zmq

실행:
	python3 calibrate_camera.py --robot lekiwi12 --remote 10.42.0.195     # 내부 코너 10x7 (칸 11x8), 칸 25 mm
	python3 calibrate_camera.py --robot lekiwi12 --remote 10.42.0.195 --cols 9 --rows 6 --square-mm 25
	→ 터미널에서 카메라를 고른다: 1 (front) · 2 (wrist) · 이름, 엔터=front

	고른 이름과 화면이 다르면 (front 인데 그리퍼가 보이면) 그 로봇은 장치가 뒤바뀌어 있다 — 로봇 설정부터 고친다.
	손목 카메라는 찍는 동안 팔을 움직이지 않는다.
	같은 로봇 스트림에 다른 클라이언트(teleop, YOLO 등)를 함께 붙이지 않는다 (프레임을 번갈아 나눠 가진다).

키: SPACE=촬영 (보드가 잡혔을 때만)  r=처음부터  q 또는 ESC=저장 없이 중단.  창은 영상의 1.5배로 열리고 드래그로 바꿀 수 있다.

결과: calib/<robot>/<front|wrist>/<front|wrist>_camera.yaml 과 찍은 원본 JPEG. 다시 찍으면 그 카메라 폴더를 덮어쓴다.
로봇에 넣는 명령은 끝날 때 출력한다 (로봇의 ~/.ros/camera_info/<front|wrist>_camera.yaml).
"""

import argparse
import base64
import glob
import json
import os
import sys
import time

import cv2
import numpy as np
import yaml

DEFAULT_CAMERA = 'front'  # 관측 JSON 의 카메라 키. ROS 이름 · 결과 파일은 '<키>_camera'
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PORT = 5556
FIRST_FRAME_TIMEOUT_S = 10.0
STALL_WARN_S = 2.0
MIN_SHOTS = 10
KEY_ESC = 27
KEY_SPACE = 32
DISPLAY_SCALE = 1.5  # 창 크기 = 영상 크기 x 이 값 (그 뒤엔 드래그로 바꿀 수 있다)
FIND_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)


def build_object_points(cols: int, rows: int, square_m: float) -> np.ndarray:
	"""보드 평면(z=0) 위 내부 코너 좌표 [m]. 순서는 findChessboardCorners 결과와 같다."""
	points = np.zeros((cols * rows, 3), np.float32)
	points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
	return points


def is_standard_jpeg(observation: dict) -> bool:
	"""ROS lekiwi_zmq_host 는 카메라 JPEG 를 그대로 싣고 _lekiwi_ros.jpeg 로 'standard' 라고 밝힌다."""
	return (observation.get('_lekiwi_ros') or {}).get('jpeg') == 'standard'


def decode_observation(observation: dict, camera: str) -> tuple[bytes, np.ndarray] | None:
	"""관측 JSON 에서 그 카메라의 (원본 JPEG, BGR 영상). 카메라가 안 실렸거나 못 풀면 None."""
	encoded = observation.get(camera)
	if not isinstance(encoded, str):
		return None
	try:
		jpeg = base64.b64decode(encoded)
	except ValueError:
		return None
	frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
	if frame is None:
		return None
	if not is_standard_jpeg(observation):
		# lerobot 호스트는 RGB 배열을 BGR 로 여기고 압축한다 (OpenCVCameraConfig 기본 RGB) → 되돌린다
		frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
	return jpeg, frame


def receive_latest(socket, poller, timeout_ms: float) -> dict | None:
	"""가장 최신 관측 하나 (CONFLATE 라 쌓이지 않는다). 시간 안에 없거나 JSON 객체가 아니면 None."""
	if not poller.poll(timeout_ms):
		return None
	try:
		observation = json.loads(socket.recv_string())
	except ValueError:
		return None
	return observation if isinstance(observation, dict) else None


def calibrate(
	obj_points: list[np.ndarray],
	img_points: list[np.ndarray],
	image_size: tuple[int, int],
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
	"""Returns:
		(RMS 재투영 오차 [px], K 3x3, 왜곡 계수 5개 (k1 k2 p1 p2 k3), 장마다 재투영 오차 [px])
	"""
	rms, k, dist, _, _, _, _, per_view = cv2.calibrateCameraExtended(
		obj_points, img_points, image_size, None, None)
	return rms, k, dist.ravel(), per_view.ravel()


def make_camera_info(k: np.ndarray, dist: np.ndarray, image_size: tuple[int, int], camera_name: str) -> dict:
	"""sensor_msgs/CameraInfo 를 채울 수 있는 camera_calibration_parsers YAML 내용."""
	assert dist.size == 5, 'plumb_bob 은 왜곡 계수 5개'
	# P 는 ROS cameracalibrator 와 같게 alpha=0 (검은 테두리 없이 잘라낸 보정 영상)
	new_k, _ = cv2.getOptimalNewCameraMatrix(k, dist, image_size, 0)
	projection = np.hstack([new_k, np.zeros((3, 1))])
	return {
		'image_width': int(image_size[0]),
		'image_height': int(image_size[1]),
		'camera_name': camera_name,
		'camera_matrix': {'rows': 3, 'cols': 3, 'data': k.ravel().tolist()},
		'distortion_model': 'plumb_bob',
		'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': dist.tolist()},
		'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).ravel().tolist()},
		'projection_matrix': {'rows': 3, 'cols': 4, 'data': projection.ravel().tolist()},
	}


def write_camera_info(path: str, info: dict, comment: str) -> None:
	"""camera_info YAML 을 쓴다. comment 는 맨 위 주석 줄로 남는다 (파서는 무시)."""
	with open(path, 'w') as file:
		file.write(f'# {comment}\n')
		yaml.safe_dump(info, file, sort_keys=False, default_flow_style=None)


def prompt_camera_name(names: list[str], default: str = DEFAULT_CAMERA) -> str:
	"""스트림에 실린 카메라 중 하나를 터미널에서 고른다 (번호 또는 이름, 엔터=기본값). 하나뿐이면 묻지 않는다."""
	fallback = default if default in names else names[0]
	if len(names) == 1:
		return names[0]
	print('[스트림에 실린 카메라]')
	for number, name in enumerate(names, 1):
		tag = '   <- 기본값' if name == fallback else ''
		print(f'  {number}) {name}{tag}')
	while True:
		raw = input(f'캘리브레이션할 카메라 (번호 또는 이름, 엔터={fallback}): ').strip()
		if raw == '':
			return fallback
		if raw.isdigit() and 1 <= int(raw) <= len(names):
			return names[int(raw) - 1]
		if raw in names:
			return raw
		print(f'  -> {raw!r} 는 없는 카메라다. {names} 중에서 고른다.')


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
	parser.add_argument('--robot', required=True, help='로봇 이름 (예: lekiwi12). 결과 폴더 이름이 된다')
	parser.add_argument('--remote', required=True, metavar='IP', help='로봇 IP (예: 10.42.0.195)')
	parser.add_argument('--port', type=int, default=DEFAULT_PORT, help='관측 스트림 포트')
	# 기본값은 쓰고 있는 보드: markhedleyjones.com Checkerboard-A4-25mm-10x7.pdf (내부 코너 10x7, 칸 11x8, A4 100% 인쇄)
	parser.add_argument('--cols', type=int, default=10, help='가로 내부 코너 수 (칸 수 - 1)')
	parser.add_argument('--rows', type=int, default=7, help='세로 내부 코너 수 (칸 수 - 1)')
	parser.add_argument('--square-mm', type=float, default=25.0, help='한 칸 길이 [mm]')
	parser.add_argument('--shots', type=int, default=20, help=f'모을 장수 (최소 {MIN_SHOTS})')
	args = parser.parse_args()
	if args.shots < MIN_SHOTS:
		parser.error(f'--shots 는 {MIN_SHOTS} 이상')
	if args.cols < 3 or args.rows < 3 or args.square_mm <= 0:
		parser.error('--cols/--rows 는 3 이상, --square-mm 는 양수')
	return args


def collect_shots(args: argparse.Namespace, socket, poller, address: str) -> tuple | None:
	"""카메라를 고르고, 화면을 띄우고 SPACE 로 장을 모은다.

	Returns:
		(camera, obj_points, img_points, jpegs, image_size, source) 또는 None (연결 실패 · 중단).
	"""
	observation = receive_latest(socket, poller, FIRST_FRAME_TIMEOUT_S * 1000.0)
	if observation is None:
		print(
			f'{address} 에서 {FIRST_FRAME_TIMEOUT_S:g} 초 안에 관측이 오지 않았다.\n'
			'  로봇: 웹 오퍼레이터에서 브링업이 떠 있는지 (카메라 켬). lekiwi_zmq_host 는 팔 · 바퀴 상태가\n'
			'        없으면 관측을 보내지 않으므로 모터까지 정상이어야 한다\n'
			'  다른 클라이언트(teleop, YOLO 등)가 같은 스트림에 붙어 있지 않은지',
			file=sys.stderr)
		return None
	# 정렬하면 1) front 2) wrist
	cameras = sorted(key for key, value in observation.items() if isinstance(value, str))
	if not cameras:
		print('스트림에 카메라가 없다. 브링업에서 카메라를 켰는지', file=sys.stderr)
		return None
	camera = prompt_camera_name(cameras)
	window = f'calibrate {camera} camera (SPACE=shot  r=reset  q=quit)'
	source = 'lekiwi_zmq_host' if is_standard_jpeg(observation) else 'lerobot lekiwi_host'
	print(f'[calibrate] {camera} @ {address} ({source})  SPACE=촬영  r=처음부터  q=중단')
	if source != 'lekiwi_zmq_host':
		print('[calibrate] 주의: lerobot 호스트 영상이다. ArUco 도 lerobot 호스트로 받을 때만 이 결과가 맞는다')

	pattern = (args.cols, args.rows)
	board_points = build_object_points(args.cols, args.rows, args.square_mm / 1000.0)
	obj_points: list[np.ndarray] = []
	img_points: list[np.ndarray] = []
	jpegs: list[bytes] = []
	image_size: tuple[int, int] | None = None
	is_found = False
	is_stalled = False
	is_window_sized = False
	gray = corners = jpeg = None
	last_frame_time = time.monotonic()
	cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
	try:
		while len(obj_points) < args.shots:
			frame_data = decode_observation(observation, camera) if observation is not None else None
			if frame_data is not None:
				jpeg, frame = frame_data
				last_frame_time = time.monotonic()
				is_stalled = False
				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				is_found, corners = cv2.findChessboardCorners(gray, pattern, FIND_FLAGS)
				if is_found:
					cv2.drawChessboardCorners(frame, pattern, corners, is_found)
				hud = f'shots {len(obj_points)}/{args.shots}  board {"OK" if is_found else "--"}'
				cv2.putText(frame, hud, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
				cv2.imshow(window, frame)
				if not is_window_sized:
					height, width = frame.shape[:2]
					cv2.resizeWindow(window, round(width * DISPLAY_SCALE), round(height * DISPLAY_SCALE))
					is_window_sized = True
			elif not is_stalled and time.monotonic() - last_frame_time > STALL_WARN_S:
				print('[calibrate] 영상이 끊겼다 — 모은 장은 그대로 두고 다시 올 때까지 기다린다')
				is_stalled = True

			key = cv2.waitKey(1) & 0xFF
			if key in (KEY_ESC, ord('q'), ord('Q')):
				print('[calibrate] 중단 — 저장하지 않았다.')
				return None
			if key == ord('r'):
				obj_points.clear()
				img_points.clear()
				jpegs.clear()
				print('[calibrate] 처음부터.')
			if key == KEY_SPACE:
				if is_stalled:
					print('[calibrate] 영상이 끊긴 동안은 찍지 않는다')
				elif not is_found:
					print(f'[calibrate] 보드가 안 잡혔다. 보드 전체가 보이는지, 내부 코너가 {args.cols}x{args.rows} '
						'(칸 수 - 1) 가 맞는지 (--cols/--rows)')
				elif jpegs and jpegs[-1] == jpeg:
					pass  # SPACE 를 누르고 있으면 같은 프레임이 여러 장 들어간다
				elif image_size is not None and gray.shape[::-1] != image_size:
					print(f'[calibrate] 영상 크기가 {image_size} → {gray.shape[::-1]} 로 바뀌었다. r 로 처음부터')
				else:
					image_size = gray.shape[::-1]
					# 화면에 보인 바로 그 프레임의 코너를 쓴다 (네트워크 지연과 상관없이 화면 = 데이터)
					obj_points.append(board_points)
					img_points.append(cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA))
					jpegs.append(jpeg)
					print(f'[calibrate] {len(obj_points)}/{args.shots}')

			observation = receive_latest(socket, poller, 30.0)
	finally:
		cv2.destroyAllWindows()
	return camera, obj_points, img_points, jpegs, image_size, source


def main() -> int:
	args = parse_args()
	try:
		import zmq  # 여기서 가져온다: pyzmq 없이도 테스트 · --help 가 돌게
	except ImportError:
		print('pyzmq 가 필요하다: sudo apt install python3-zmq', file=sys.stderr)
		return 1

	address = f'tcp://{args.remote}:{args.port}'
	context = zmq.Context()
	socket = context.socket(zmq.PULL)
	socket.setsockopt(zmq.CONFLATE, 1)
	socket.connect(address)
	poller = zmq.Poller()
	poller.register(socket, zmq.POLLIN)
	try:
		shots = collect_shots(args, socket, poller, address)
	finally:
		socket.close(linger=0)
		context.term()
	if shots is None:
		return 1
	camera, obj_points, img_points, jpegs, image_size, source = shots
	camera_name = f'{camera}_camera'  # ROS 카메라 노드 · camera_info 이름 (front_camera, wrist_camera)

	print('[calibrate] 계산 중...')
	rms, k, dist, per_view = calibrate(obj_points, img_points, image_size)
	stamp = time.strftime('%Y%m%d_%H%M%S')
	out_dir = os.path.join(BASE_DIR, 'calib', args.robot, camera)
	os.makedirs(out_dir, exist_ok=True)
	for old in glob.glob(os.path.join(out_dir, '*.jpg')):  # 지난번 장이 섞이지 않게 (장수가 다를 수 있다)
		os.remove(old)
	for index, jpeg in enumerate(jpegs):
		with open(os.path.join(out_dir, f'{index:02d}.jpg'), 'wb') as file:
			file.write(jpeg)
	yaml_path = os.path.join(out_dir, f'{camera_name}.yaml')
	write_camera_info(
		yaml_path, make_camera_info(k, dist, image_size, camera_name),
		f'{args.robot} {camera_name} {stamp}: RMS {rms:.3f} px, {len(obj_points)} shots, '
		f'board {args.cols}x{args.rows} inner corners @ {args.square_mm:g} mm, via {source}')

	worst = np.argsort(per_view)[::-1][:3]
	print(f'RMS 재투영 오차: {rms:.3f} px  (0.5 이하 좋음, 1.0 넘으면 다시 찍기)')
	print('오차 큰 장: ' + ', '.join(f'{i:02d}.jpg {per_view[i]:.2f}px' for i in worst))
	print(f'K =\n{k}\ndist = {dist}')
	print(f'저장: {yaml_path}')
	print('로봇에 넣기:')
	print(f"  ssh roboseasy@{args.remote} 'mkdir -p ~/.ros/camera_info'")
	print(f'  scp {yaml_path} roboseasy@{args.remote}:.ros/camera_info/{camera_name}.yaml')
	return 0


if __name__ == '__main__':
	sys.exit(main())
