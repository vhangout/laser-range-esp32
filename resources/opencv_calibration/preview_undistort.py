import argparse
import json
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview live undistorted image from ESP32-CAM stream."
    )
    parser.add_argument(
        "--url",
        default="http://192.168.1.73/rawcam",
        help="Raw JPEG endpoint URL.",
    )
    parser.add_argument(
        "--calibration",
        required=True,
        help="Path to calibration JSON file.",
    )
    return parser.parse_args()


def fetch_jpeg_frame(url: str, timeout_s: float = 3.0) -> np.ndarray:
    with urlopen(url, timeout=timeout_s) as response:
        data = response.read()

    buffer = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("Failed to decode JPEG frame.")
    return frame


def load_calibration(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = json.loads(path.read_text(encoding="utf-8"))
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
    return camera_matrix, dist_coeffs


def main() -> None:
    args = parse_args()
    calibration_path = Path(args.calibration)
    camera_matrix, dist_coeffs = load_calibration(calibration_path)

    print(f"Using calibration: {calibration_path}")
    print("Controls:")
    print("  q or esc - exit")

    while True:
        try:
            frame = fetch_jpeg_frame(args.url)
            h, w = frame.shape[:2]
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeffs, (w, h), 1, (w, h)
            )
            undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)

            x, y, rw, rh = roi
            if rw > 0 and rh > 0:
                undistorted = undistorted[y : y + rh, x : x + rw]

            cv2.imshow("Raw", frame)
            cv2.imshow("Undistorted", undistorted)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
        except URLError as exc:
            print(f"Cannot fetch stream: {exc}. Retrying...")
            if cv2.waitKey(200) & 0xFF in (27, ord("q")):
                break
        except Exception as exc:
            print(f"Unexpected error: {exc}")
            if cv2.waitKey(200) & 0xFF in (27, ord("q")):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
