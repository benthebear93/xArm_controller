"""Create an A4 vector ChArUco PDF and matching OpenCV board metadata.

Requires OpenCV with cv2.aruco, NumPy, and ReportLab. No camera or robot access.
Run from the repository root: python3 assets/calibration/generate_charuco.py
"""

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "output/pdf/charuco_a4_30mm.pdf"
COLUMNS, ROWS = 6, 9
SQUARE_MM, MARKER_MM = 30.0, 22.5
PAPER_MM = (210.0, 297.0)
DICTIONARY = "DICT_4X4_50"


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY))
    board = cv2.aruco.CharucoBoard((COLUMNS, ROWS), SQUARE_MM, MARKER_MM, dictionary)
    board.setLegacyPattern(False)
    ids = board.getIds().flatten().tolist()
    width, height = COLUMNS * SQUARE_MM, ROWS * SQUARE_MM
    left, top = (PAPER_MM[0] - width) / 2, (PAPER_MM[1] - height) / 2
    reference = board.generateImage((COLUMNS * 40, ROWS * 40), marginSize=0, borderBits=1)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(str(OUTPUT), pagesize=tuple(v * mm for v in PAPER_MM), pageCompression=1)
    pdf.setTitle("A4 ChArUco - 6 x 9 - 30 mm squares - DICT_4X4_50")
    pdf.setAuthor("xArm Controller")
    pdf.setFillColorRGB(0, 0, 0)

    def rectangle(x, y, w, h, path=None):
        # Board coordinates: x right, y down, origin at the pattern's top-left.
        coordinates = ((left + x) * mm, (PAPER_MM[1] - top - y - h) * mm,
                       w * mm, h * mm)
        if path is None:
            pdf.rect(*coordinates, stroke=0, fill=1)
        else:
            path.rect(*coordinates)

    # Match the exact chessboard parity returned by the installed OpenCV.
    for row in range(ROWS):
        for column in range(COLUMNS):
            if reference[row * 40 + 1, column * 40 + 1] == 0:
                rectangle(column * SQUARE_MM, row * SQUARE_MM, SQUARE_MM, SQUARE_MM)

    # Fill each marker once as a compound vector path. Painting adjacent rows
    # separately creates stitching lines in some antialiasing PDF renderers.
    module_count = dictionary.markerSize + 2
    for marker_id, corners in zip(ids, board.getObjPoints()):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, module_count, borderBits=1)
        x0, y0 = np.asarray(corners)[0, :2]
        module = MARKER_MM / module_count
        marker_path = pdf.beginPath()
        for row in range(module_count):
            column = 0
            while column < module_count:
                if marker[row, column] != 0:
                    column += 1
                    continue
                end = column + 1
                while end < module_count and marker[row, end] == 0:
                    end += 1
                rectangle(float(x0) + column * module, float(y0) + row * module,
                          (end - column) * module, module, path=marker_path)
                column = end
        pdf.drawPath(marker_path, stroke=0, fill=1, fillMode=1)

    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(PAPER_MM[0] / 2 * mm, 289 * mm,
                         "ChArUco 6 x 9 | Square 30 mm | Marker 22.5 mm | DICT_4X4_50")
    pdf.drawCentredString(PAPER_MM[0] / 2 * mm, 7 * mm,
                         "Print at 100% / Actual Size | Legacy OFF | IDs 0-26 | Measure a square before use")
    pdf.save()

    metadata = {
        "target_type": "charuco",
        "pdf_file": OUTPUT.name,
        "sha256": hashlib.sha256(OUTPUT.read_bytes()).hexdigest(),
        "source": "Generated locally with OpenCV; vector PDF written with ReportLab",
        "pdf_marker_rendering": "Single compound vector fill per marker; nonzero winding",
        "generator": "assets/calibration/generate_charuco.py",
        "reference_url": "https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html",
        "generated_with_opencv": cv2.__version__,
        "paper_size_mm": list(PAPER_MM),
        "pattern_size_mm": [width, height],
        "margins_left_top_right_bottom_mm": [left, top, left, top],
        "squares_columns_rows": [COLUMNS, ROWS],
        "square_size_mm": SQUARE_MM,
        "marker_size_mm": MARKER_MM,
        "dictionary": DICTIONARY,
        "legacy_pattern": False,
        "marker_border_bits": 1,
        "start_id": 0,
        "marker_ids": ids,
        "marker_count": len(ids),
        "charuco_corner_count": len(board.getChessboardCorners()),
        "opencv_square_length_m": SQUARE_MM / 1000,
        "opencv_marker_length_m": MARKER_MM / 1000,
        "opencv_size_columns_rows": [COLUMNS, ROWS],
        "print_scale_percent": 100,
        "setup": "Fixed external camera; board rigidly attached to the robot tool.",
    }
    OUTPUT.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(OUTPUT)
    print(f"OpenCV {cv2.__version__}: {len(ids)} markers, {len(board.getChessboardCorners())} ChArUco corners")


if __name__ == "__main__":
    main()
