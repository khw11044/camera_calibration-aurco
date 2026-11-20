#!/usr/bin/env python3
"""LeKiwi 카메라로 ArUco 마커가 인식되는지 화면으로 확인한다 — PC 에서 실행한다.

캘리브레이션 도구(../camera_calibration/calibrate_camera.py)와 같은 경로로 로봇 영상을 받아
(ZMQ 5556, 1) front  2) wrist 고르기) 잡힌 마커마다 테두리 · 사전 · ID 를 그린다.
캘리브레이션 결과(--robot)를 주면 좌표축과 카메라에서 마커 중심까지 거리도 그린다.

기본값은 수업용 마커 (aruco_maker/make_aruco_a4_20mm.py): DICT_4X4_50, ID 0 · 1, 한 변 20 mm.

준비: 캘리브레이션 도구와 같다 — 로봇은 웹 오퍼레이터에서 브링업 시작 (카메라 켬), PC 는 sudo apt install python3-zmq

실행:
	python3 aruco_viewer.py --remote 10.42.0.195                     # ID 만
	python3 aruco_viewer.py --remote 10.42.0.195 --robot lekiwi12    # + 거리 · 좌표축
	python3 aruco_viewer.py --remote 10.42.0.195 --dict auto         # 사전을 모르는 마커

	--robot 은 ../camera_calibration/calib/<robot>/<front|wrist>/ 의 캘리브레이션 결과를 쓴다.
	--marker-mm 은 인쇄한 마커의 검은 테두리 바깥 한 변 (흰 여백 제외). 100% 로 인쇄했는지 페이지 아래 100 mm 막대로 확인한다.
	--dict auto 는 주요 계열 9개(4X4~7X7, ARUCO_ORIGINAL, AprilTag)를 매 프레임 모두 훑는다. 잡히면 라벨에 사전이 나온다.

키: q 또는 ESC=끝내기.  창은 영상의 1.5배로 열리고 드래그로 바꿀 수 있다.
"""

import argparse
import os
import sys
import time
from collections import Counter

import cv2
import numpy as np
import yaml

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_DIR = os.path.join(BASE_DIR, '..', 'camera_calibration')
sys.path.insert(0, CALIBRATION_DIR)
# 영상 받기 · 카메라 고르기는 캘리브레이션 도구 것을 그대로 쓴다 (같은 경로로 받아야 캘리브레이션 값이 맞는다)
from calibrate_camera import (  # noqa: E402
	DEFAULT_PORT,
	DISPLAY_SCALE,
	FIRST_FRAME_TIMEOUT_S,
	KEY_ESC,
	STALL_WARN_S,
	decode_observation,
	prompt_camera_name,
	receive_latest,
)

# --dict auto 로 훑는 계열. 각 계열의 _1000 은 _50 · _100 · _250 을 앞부분으로 포함한다 (ID 가 같다)
AUTO_DICTS = (
	'4X4_1000', '5X5_1000', '6X6_1000', '7X7_1000', 'ARUCO_ORIGINAL',
	'APRILTAG_16h5', 'APRILTAG_25h9', 'APRILTAG_36h10', 'APRILTAG_36h11')
# 수업용 마커 (aruco_maker/make_aruco_a4_20mm.py): DICT_4X4_50, ID 0 · 1, 검은 테두리 바깥 한 변 20 mm
DEFAULT_DICT = '4X4_50'
DEFAULT_MARKER_MM = 20.0
LABEL_COLOR = (0, 255, 0)
HUD_COLOR = (0, 255, 255)


def dict_id(name: str) -> int | None:
	"""'4x4_50' · 'DICT_4X4_50' · 'apriltag_36h11' → cv2.aruco 사전 번호. 없는 이름이면 None."""
	base = name.upper().removeprefix('DICT_')
	for candidate in (base, base.replace('H', 'h')):  # AprilTag 이름은 36h11 처럼 소문자 h
		value = getattr(cv2.aruco, f'DICT_{candidate}', None)
		if value is not None:
			return value
	return None


def make_detector(dictionary_id: int):
	"""사전 하나의 검출 함수 gray → (corners, ids). ids 는 없으면 None."""
	dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
	is_new_api = hasattr(cv2.aruco, 'ArucoDetector')  # OpenCV 4.7+. Ubuntu 24.04 apt 의 4.6 은 옛 함수만 있다
	parameters = cv2.aruco.DetectorParameters() if is_new_api else cv2.aruco.DetectorParameters_create()
	# 코너를 서브픽셀로 다듬는다. 기본값(NONE)이면 거리가 1% 안팎 틀어진다 (test_aruco_viewer 에서 확인)
	parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
	if is_new_api:
		detector = cv2.aruco.ArucoDetector(dictionary, parameters)
		detect = lambda gray: detector.detectMarkers(gray)[:2]  # noqa: E731
	else:
		detect = lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)[:2]  # noqa: E731

	def detect_or_none(gray: np.ndarray) -> tuple:
		corners, ids = detect(gray)
		# 4.6 은 못 찾으면 None, 새 버전(5.x)은 빈 배열을 준다 → None 으로 맞춘다
		return corners, (ids if ids is not None and len(ids) else None)

	return detect_or_none


def marker_pose(
	corners: np.ndarray, marker_m: float, k: np.ndarray, dist: np.ndarray,
) -> tuple[np.ndarray, np.ndarray] | None:
	"""마커 네 꼭짓점 (검출 순서: 왼위 · 오른위 · 오른아래 · 왼아래) → 카메라 기준 (rvec, tvec [m]). 실패하면 None."""
	half = marker_m / 2.0
	object_points = np.array(
		[[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)
	is_ok, rvec, tvec = cv2.solvePnP(
		object_points, corners.reshape(4, 2).astype(np.float32), k, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
	return (rvec, tvec) if is_ok else None


def calibration_path(calib_root: str, robot: str, camera: str) -> str | None:
	"""calib/<robot>/<camera>/<camera>_camera.yaml. 없으면 None."""
	path = os.path.join(calib_root, robot, camera, f'{camera}_camera.yaml')
	return path if os.path.isfile(path) else None


def load_camera_info(path: str) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
	"""camera_info YAML → (K 3x3, 왜곡 계수, (가로, 세로))."""
	with open(path) as file:
		info = yaml.safe_load(file)
	k = np.array(info['camera_matrix']['data'], float).reshape(3, 3)
	dist = np.array(info['distortion_coefficients']['data'], float)
	return k, dist, (int(info['image_width']), int(info['image_height']))


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
	parser.add_argument('--remote', required=True, metavar='IP', help='로봇 IP (예: 10.42.0.195)')
	parser.add_argument('--port', type=int, default=DEFAULT_PORT, help='관측 스트림 포트')
	parser.add_argument('--robot', help='로봇 이름 (예: lekiwi12). 주면 그 로봇의 캘리브레이션 결과를 쓴다')
	parser.add_argument(
		'--marker-mm', type=float, default=DEFAULT_MARKER_MM, help='마커 한 변 [mm] (검은 테두리 바깥, 흰 여백 제외)')
	parser.add_argument(
		'--dict', default=DEFAULT_DICT, help='사전 (예: 4X4_50, 6X6_250, APRILTAG_36h11). auto=주요 계열 모두')
	args = parser.parse_args()
	if args.dict != 'auto' and dict_id(args.dict) is None:
		names = sorted(name.removeprefix('DICT_') for name in dir(cv2.aruco) if name.startswith('DICT_'))
		parser.error(f'없는 사전 {args.dict!r}. 고를 수 있는 것: {", ".join(names)}')
	if args.marker_mm <= 0:
		parser.error('--marker-mm 은 양수')
	return args


def view(args: argparse.Namespace, socket, poller, address: str) -> int:
	"""카메라를 고르고, 마커를 그려 보여 준다. q · ESC 까지."""
	names = AUTO_DICTS if args.dict == 'auto' else (args.dict,)
	detectors = [(name, make_detector(dict_id(name))) for name in names]

	observation = receive_latest(socket, poller, FIRST_FRAME_TIMEOUT_S * 1000.0)
	if observation is None:
		print(
			f'{address} 에서 {FIRST_FRAME_TIMEOUT_S:g} 초 안에 관측이 오지 않았다.\n'
			'  로봇: 웹 오퍼레이터에서 브링업이 떠 있는지 (카메라 켬, 모터까지 정상)\n'
			'  다른 클라이언트(teleop, YOLO, 캘리브레이션 등)가 같은 스트림에 붙어 있지 않은지',
			file=sys.stderr)
		return 1
	cameras = sorted(key for key, value in observation.items() if isinstance(value, str))
	if not cameras:
		print('스트림에 카메라가 없다. 브링업에서 카메라를 켰는지', file=sys.stderr)
		return 1
	camera = prompt_camera_name(cameras)

	k = dist = calib_size = None
	if args.robot:
		path = calibration_path(os.path.join(CALIBRATION_DIR, 'calib'), args.robot, camera)
		if path is None:
			print(f'[aruco] {args.robot} {camera} 캘리브레이션 결과가 없다 — ID 만 보여 준다')
		else:
			k, dist, calib_size = load_camera_info(path)
			print(f'[aruco] 캘리브레이션: {os.path.relpath(path, BASE_DIR)}')
	is_pose = k is not None
	marker_m = args.marker_mm / 1000.0
	if not is_pose:
		print('[aruco] 거리 · 좌표축은 끔 (켜려면 --robot <로봇 이름>)')
	print(f'[aruco] {camera} @ {address}  사전 {args.dict}  마커 {args.marker_mm:g} mm  q=끝내기')

	window = f'aruco {camera} (q=quit)'
	cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
	is_window_sized = False
	is_stalled = False
	last_frame_time = time.monotonic()
	last_ids: set | None = None
	try:
		while True:
			frame_data = decode_observation(observation, camera) if observation is not None else None
			if frame_data is not None:
				_, frame = frame_data
				last_frame_time = time.monotonic()
				is_stalled = False
				if is_pose and frame.shape[1::-1] != calib_size:
					print(f'[aruco] 영상 {frame.shape[1::-1]} 와 캘리브레이션 {calib_size} 크기가 달라 거리는 끈다')
					is_pose = False
				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

				detections: list[tuple[tuple[str, int], str]] = []  # ((사전, ID), 라벨). 같은 ID 가 여러 장일 수 있다
				for name, detect in detectors:
					corners, ids = detect(gray)
					if ids is None:
						continue
					cv2.aruco.drawDetectedMarkers(frame, corners)
					for marker_corners, marker_id in zip(corners, ids.ravel()):
						label = f'{name} id {marker_id}'
						if is_pose:
							pose = marker_pose(marker_corners, marker_m, k, dist)
							if pose is not None:
								cv2.drawFrameAxes(frame, k, dist, pose[0], pose[1], marker_m * 0.5)
								label += f'  {np.linalg.norm(pose[1]):.2f} m'
						x, y = marker_corners.reshape(4, 2)[0]  # 왼위 꼭짓점 위에 쓴다
						cv2.putText(frame, label, (int(x), max(int(y) - 10, 15)), cv2.FONT_HERSHEY_SIMPLEX,
							0.6, LABEL_COLOR, 2, cv2.LINE_AA)
						detections.append(((name, int(marker_id)), label))

				counts = Counter(key for key, _ in detections)
				if set(counts) != last_ids:  # 보이는 ID 가 바뀔 때만 출력 (장수가 깜빡이는 건 무시)
					summary = [
						label if counts[key] == 1 else f'{key[0]} id {key[1]} x{counts[key]}'
						for key, label in sorted(dict(detections).items())]
					print('[aruco] 보임: ' + (', '.join(summary) or '없음'))
					last_ids = set(counts)
				hud = f'{camera}  dict {args.dict}  markers {len(detections)}' + ('' if is_pose else '  (no distance)')
				cv2.putText(frame, hud, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, HUD_COLOR, 2, cv2.LINE_AA)
				cv2.imshow(window, frame)
				if not is_window_sized:
					height, width = frame.shape[:2]
					cv2.resizeWindow(window, round(width * DISPLAY_SCALE), round(height * DISPLAY_SCALE))
					is_window_sized = True
			elif not is_stalled and time.monotonic() - last_frame_time > STALL_WARN_S:
				print('[aruco] 영상이 끊겼다 — 화면은 마지막 프레임이다. 다시 올 때까지 기다린다')
				is_stalled = True

			if (cv2.waitKey(1) & 0xFF) in (KEY_ESC, ord('q'), ord('Q')):
				return 0
			observation = receive_latest(socket, poller, 30.0)
	finally:
		cv2.destroyAllWindows()


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
		return view(args, socket, poller, address)
	finally:
		socket.close(linger=0)
		context.term()


if __name__ == '__main__':
	sys.exit(main())
