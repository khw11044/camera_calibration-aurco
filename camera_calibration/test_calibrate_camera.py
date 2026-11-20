"""calibrate_camera 자체 점검 (카메라 · 로봇 · pyzmq 없이).

알려진 K · 왜곡으로 가상 보드 20장을 투영해 calibrate 가 그 값을 되찾는지, 저장한 YAML 을 다시 읽으면
같은 값인지, 관측 JSON 의 영상을 호스트 종류에 맞게 푸는지, 카메라 고르기 입력을 맞게 받는지 본다.
실행: python3 test_calibrate_camera.py  (pytest 로도 돈다)
"""

import base64
import builtins
import os
import tempfile

import cv2
import numpy as np
import yaml

from calibrate_camera import (
	build_object_points,
	calibrate,
	decode_observation,
	make_camera_info,
	prompt_camera_name,
	write_camera_info,
)

K_TRUE = np.array([[600.0, 0.0, 320.0], [0.0, 605.0, 240.0], [0.0, 0.0, 1.0]])
DIST_TRUE = np.array([-0.25, 0.08, 0.001, -0.0015, 0.0])
IMAGE_SIZE = (640, 480)


def make_views(count: int = 20) -> tuple[list[np.ndarray], list[np.ndarray]]:
	"""보드 중심을 카메라 앞 0.35~0.6 m 에 여러 각도로 둔 가상 촬영 (0.1 px 잡음)."""
	rng = np.random.default_rng(0)
	board = build_object_points(13, 9, 0.02)
	board_centered = board - board.mean(axis=0)
	obj_points, img_points = [], []
	for _ in range(count):
		rvec = rng.uniform(-0.6, 0.6, 3)
		tvec = np.array([rng.uniform(-0.06, 0.06), rng.uniform(-0.04, 0.04), rng.uniform(0.35, 0.6)])
		projected, _ = cv2.projectPoints(board_centered, rvec, tvec, K_TRUE, DIST_TRUE)
		projected += rng.normal(0.0, 0.1, projected.shape)
		obj_points.append(board_centered.astype(np.float32))
		img_points.append(projected.astype(np.float32))
	return obj_points, img_points


def test_calibrate_recovers_intrinsics() -> None:
	rms, k, dist, per_view = calibrate(*make_views(), IMAGE_SIZE)
	assert rms < 0.3, rms
	assert per_view.shape == (20,)
	assert np.allclose(k, K_TRUE, atol=3.0), k
	assert np.allclose(dist[:2], DIST_TRUE[:2], atol=0.03), dist


def test_camera_info_yaml_round_trip() -> None:
	_, k, dist, _ = calibrate(*make_views(), IMAGE_SIZE)
	with tempfile.TemporaryDirectory() as tmp:
		path = os.path.join(tmp, 'front_camera.yaml')
		write_camera_info(path, make_camera_info(k, dist, IMAGE_SIZE, 'wrist_camera'), 'test')
		with open(path) as file:
			text = file.read()
	info = yaml.safe_load(text)
	assert text.startswith('# test\n')
	assert 'data: [' in text  # ROS 파일처럼 행렬은 한 줄 리스트
	assert (info['image_width'], info['image_height']) == IMAGE_SIZE
	assert info['camera_name'] == 'wrist_camera'
	assert info['distortion_model'] == 'plumb_bob'
	assert np.allclose(np.reshape(info['camera_matrix']['data'], (3, 3)), k)
	assert np.allclose(info['distortion_coefficients']['data'], dist)
	assert len(info['projection_matrix']['data']) == 12
	assert info['rectification_matrix']['data'] == np.eye(3).ravel().tolist()


def test_decode_observation_by_host() -> None:
	blue = np.zeros((48, 64, 3), np.uint8)
	blue[:, :, 0] = 255  # BGR 의 B
	jpeg = cv2.imencode('.jpg', blue)[1].tobytes()
	encoded = base64.b64encode(jpeg).decode('ascii')

	raw, frame = decode_observation({'front': encoded, '_lekiwi_ros': {'jpeg': 'standard'}}, 'front')
	assert raw == jpeg
	assert frame[..., 0].mean() > 200 and frame[..., 2].mean() < 50  # ROS 호스트: 그대로
	_, frame = decode_observation({'front': encoded, 'arm_gripper.pos': 0.0}, 'front')
	assert frame[..., 2].mean() > 200 and frame[..., 0].mean() < 50  # lerobot 호스트: R/B 되돌림

	assert decode_observation({'wrist': encoded}, 'front') is None
	assert decode_observation({'front': 1.0}, 'front') is None
	assert decode_observation({'front': 'a'}, 'front') is None          # base64 가 아님
	assert decode_observation({'front': 'aGVsbG8='}, 'front') is None   # JPEG 가 아님


def test_prompt_camera_name() -> None:
	names = ['front', 'wrist']
	real_input = builtins.input
	try:
		for answers, expected in ((['1'], 'front'), (['2'], 'wrist'), ([''], 'front'), (['wrist'], 'wrist'),
				(['3', 'x', '2'], 'wrist')):
			replies = iter(answers)
			builtins.input = lambda _prompt: next(replies)
			assert prompt_camera_name(names) == expected, answers
		builtins.input = lambda _prompt: 1 / 0  # 하나뿐이면 묻지 않는다
		assert prompt_camera_name(['wrist']) == 'wrist'
	finally:
		builtins.input = real_input


if __name__ == '__main__':
	test_calibrate_recovers_intrinsics()
	test_camera_info_yaml_round_trip()
	test_decode_observation_by_host()
	test_prompt_camera_name()
	print('ok')
