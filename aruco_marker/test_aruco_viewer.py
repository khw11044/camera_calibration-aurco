"""aruco_viewer 자체 점검 (카메라 · 로봇 · pyzmq 없이).

알려진 자세로 마커를 가상 카메라 영상에 그려 넣고, 맞는 사전으로만 잡히는지 · 거리를 되찾는지,
사전 이름을 맞게 풀이하는지, 캘리브레이션 파일을 찾아 읽는지 본다.
실행: python3 test_aruco_viewer.py  (pytest 로도 돈다)
"""

import os
import tempfile

import cv2
import numpy as np

from aruco_viewer import AUTO_DICTS, DEFAULT_DICT, dict_id, calibration_path, load_camera_info, make_detector, marker_pose
from calibrate_camera import make_camera_info, write_camera_info  # aruco_viewer 가 경로를 잡아 둔다

K = np.array([[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]])
DIST = np.zeros(5)
MARKER_M = 0.10
MARKER_PX = 200
QUIET_PX = 40  # 마커 둘레 흰 여백


def render_scene(marker_id: int, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
	"""DICT_4X4_50 마커(흰 여백 포함)를 그 자세로 640x480 회색 영상에 투영한다."""
	dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
	generate = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
	image = cv2.copyMakeBorder(
		generate(dictionary, marker_id, MARKER_PX), QUIET_PX, QUIET_PX, QUIET_PX, QUIET_PX,
		cv2.BORDER_CONSTANT, value=255)
	side = image.shape[0]
	half = MARKER_M / 2.0 * side / MARKER_PX  # 여백까지 포함한 반 변 [m]
	board = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
	projected, _ = cv2.projectPoints(board, rvec, tvec, K, DIST)
	source = np.float32([[0, 0], [side, 0], [side, side], [0, side]])
	homography = cv2.getPerspectiveTransform(source, projected.reshape(4, 2).astype(np.float32))
	return cv2.warpPerspective(image, homography, (640, 480), borderValue=128)


def facing_pose(tilt: tuple[float, float, float], tvec: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray]:
	"""카메라를 마주 본 마커 (마커 z 가 카메라 쪽, OpenCV ArUco 관례) 에 조금 기울임을 더한 자세."""
	facing, _ = cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))
	tilted, _ = cv2.Rodrigues(np.array(tilt))
	rvec, _ = cv2.Rodrigues(facing @ tilted)
	return rvec, np.array(tvec, float)


def test_dict_names() -> None:
	assert dict_id('4x4_50') == cv2.aruco.DICT_4X4_50
	assert dict_id('DICT_4X4_50') == cv2.aruco.DICT_4X4_50
	assert dict_id('apriltag_36h11') == cv2.aruco.DICT_APRILTAG_36h11
	assert dict_id('APRILTAG_36H11') == cv2.aruco.DICT_APRILTAG_36h11
	assert dict_id('nope') is None
	assert dict_id(DEFAULT_DICT) == cv2.aruco.DICT_4X4_50  # 수업용 마커 사전
	assert all(dict_id(name) is not None for name in AUTO_DICTS)


def test_detects_marker_and_distance() -> None:
	rvec, tvec = facing_pose((0.3, -0.25, 0.1), (0.05, -0.03, 0.6))
	gray = render_scene(7, rvec, tvec)

	corners, ids = make_detector(dict_id('4X4_50'))(gray)
	assert ids is not None and ids.ravel().tolist() == [7], ids
	pose = marker_pose(corners[0], MARKER_M, K, DIST)
	assert pose is not None
	assert abs(np.linalg.norm(pose[1]) - np.linalg.norm(tvec)) < 0.01 * np.linalg.norm(tvec), pose[1].ravel()

	# 수업용 ID 0 · 1 도 기본 사전으로 잡힌다
	for marker_id in (0, 1):
		ids = make_detector(dict_id(DEFAULT_DICT))(render_scene(marker_id, rvec, tvec))[1]
		assert ids is not None and ids.ravel().tolist() == [marker_id], (marker_id, ids)

	# auto 로 훑으면 4X4 계열에서만, 같은 ID 로 잡힌다
	found = {name: make_detector(dict_id(name))(gray)[1] for name in AUTO_DICTS}
	hits = {name: ids.ravel().tolist() for name, ids in found.items() if ids is not None}
	assert hits == {'4X4_1000': [7]}, hits


def test_calibration_path_round_trip() -> None:
	k_saved = np.array([[600.0, 0.0, 330.0], [0.0, 601.0, 250.0], [0.0, 0.0, 1.0]])
	dist_saved = np.array([0.1, -0.2, 0.001, 0.002, 0.05])
	with tempfile.TemporaryDirectory() as root:
		folder = os.path.join(root, 'lekiwi12', 'front')
		os.makedirs(folder)
		write_camera_info(
			os.path.join(folder, 'front_camera.yaml'),
			make_camera_info(k_saved, dist_saved, (640, 480), 'front_camera'), 'test')
		os.makedirs(os.path.join(root, 'lekiwi12', 'wrist'))
		path = calibration_path(root, 'lekiwi12', 'front')
		assert path == os.path.join(folder, 'front_camera.yaml'), path
		assert calibration_path(root, 'lekiwi12', 'wrist') is None  # 폴더만 있고 YAML 없음
		assert calibration_path(root, 'lekiwi99', 'front') is None
		k, dist, size = load_camera_info(path)
	assert np.allclose(k, k_saved) and np.allclose(dist, dist_saved) and size == (640, 480)


if __name__ == '__main__':
	test_dict_names()
	test_detects_marker_and_distance()
	test_calibration_path_round_trip()
	print('ok')
