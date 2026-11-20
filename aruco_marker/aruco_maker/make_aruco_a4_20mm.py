"""수업용: ArUco 마커 ID 0~4 를 각각 30장씩 A4 5쪽 PDF로 만든다.

한 쪽에 한 ID: 1쪽 = ID 0 × 30, 2쪽 = ID 1 × 30, … 5쪽 = ID 4 × 30. 사전은 DICT_4X4_50, 검은 테두리 기준 한 변 20mm.

    pip install opencv-contrib-python pillow
    python make_aruco_a4_20mm.py

인쇄 배율은 반드시 '실제 크기(100%)'. '페이지에 맞춤'이면 마커가 20mm보다 작아진다.
인쇄 후 페이지 하단 막대가 100mm인지 자로 확인할 것.
"""
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DICT = cv2.aruco.DICT_4X4_50  # wikidocs 6-12 예제와 같은 사전
STUDENTS = 30
# 5쪽 = ID 0~4 각 30장
MARKER_IDS = range(5)        # ID 0~4
IDS = [i for i in MARKER_IDS for _ in range(STUDENTS)]
MARKER_MM = 20
OUT = "aruco_id0-4_x30_20mm_A4.pdf"

PX_PER_MM = 24                # 609.6 dpi: A4·20mm·마커 한 칸(20/6mm)이 모두 정수 픽셀로 떨어진다
PAGE_MM = (210, 297)
GAP_MM = 10                   # 마커 사이 흰 여백. 가운데를 자르면 마커 둘레에 5mm(3칸) 흰 테두리가 남는다
LABEL_MM = 5
COLS, ROWS = 5, 6             # 페이지당 30개 = 한 ID 를 한 페이지에
TOP_MM = 15


def mm(v):
    return round(v * PX_PER_MM)


aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)
cells = aruco_dict.markerSize + 2  # 비트 격자 + 양쪽 검은 테두리 1칸
# OpenCV 4.7+ 는 generateImageMarker, 4.6 이하는 drawMarker
gen = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
font = ImageFont.load_default(size=mm(3.5))
font_small = ImageFont.load_default(size=mm(2.5))


def marker(i):
    # 칸당 1px 로 뽑아 최근접 확대 → 경계가 흐려지지 않는다
    return Image.fromarray(gen(aruco_dict, i, cells)).resize((mm(MARKER_MM),) * 2, Image.NEAREST)


def detect(gray):
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(aruco_dict).detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, aruco_dict)


x0 = (PAGE_MM[0] - (COLS * MARKER_MM + (COLS - 1) * GAP_MM)) / 2
per_page = COLS * ROWS
pages = []
for p in range(0, len(IDS), per_page):
    chunk = IDS[p:p + per_page]
    page = Image.new("L", (mm(PAGE_MM[0]), mm(PAGE_MM[1])), 255)
    d = ImageDraw.Draw(page)
    for k, i in enumerate(chunk):
        r, c = divmod(k, COLS)
        x = mm(x0 + c * (MARKER_MM + GAP_MM))
        y = mm(TOP_MM + r * (MARKER_MM + LABEL_MM + GAP_MM))
        page.paste(marker(i), (x, y))
        d.text((x + mm(MARKER_MM) // 2, y + mm(MARKER_MM + 1)), f"ID {i}", fill=0, font=font, anchor="mt")

    # 축척 확인용 100mm 막대
    by = mm(PAGE_MM[1] - 15)
    d.rectangle([mm(x0), by, mm(x0 + 100) - 1, by + mm(1) - 1], fill=0)
    d.text((mm(x0), by + mm(2)), f"Bar = 100 mm. Print at 100% (Actual size).  DICT_4X4_50, marker {MARKER_MM} mm",
           fill=0, font=font_small)

    # 자체 검사: 페이지에 넣은 마커가 개수까지 전부 검출되고, 한 변이 MARKER_MM 인지
    corners, found, _ = detect(np.asarray(page))
    assert found is not None and sorted(found.ravel()) == sorted(chunk), (found, chunk)
    for cn in corners:
        side_mm = np.linalg.norm(cn[0][0] - cn[0][1]) / PX_PER_MM
        assert abs(side_mm - MARKER_MM) < 0.2, side_mm

    pages.append(page.convert("1", dither=Image.Dither.NONE))  # 1비트 → PDF 에 무손실(CCITT)로 들어감

pages[0].save(OUT, save_all=True, append_images=pages[1:], resolution=PX_PER_MM * 25.4)
print(f"{OUT}: {len(pages)}쪽, ID 0~{MARKER_IDS[-1]} 각 {STUDENTS}장, 한 변 {MARKER_MM}mm")
