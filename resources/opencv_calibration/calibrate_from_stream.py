import argparse
import json
from datetime import datetime
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture ChArUco frames from ESP32-CAM stream and calibrate camera."
    )
    parser.add_argument(
        "--url",
        default="http://192.168.1.73/rawcam",
        help="Raw JPEG endpoint URL.",
    )
    parser.add_argument(
        "--squares-x",
        type=int,
        default=10,
        help="Number of chessboard squares along X for the ChArUco board.",
    )
    parser.add_argument(
        "--squares-y",
        type=int,
        default=7,
        help="Number of chessboard squares along Y for the ChArUco board.",
    )
    parser.add_argument(
        "--square-length",
        type=float,
        default=17.0,
        help="Chess square side length in mm (or any consistent unit).",
    )
    parser.add_argument(
        "--marker-length",
        type=float,
        default=12.0,
        help="ArUco marker side length in the same unit as square length.",
    )
    parser.add_argument(
        "--dictionary",
        default="DICT_6X6_250",
        help="ArUco dictionary name from cv2.aruco, for example DICT_6X6_250.",
    )
    parser.add_argument(
        "--min-frames",
        type=int,
        default=12,
        help="Minimum accepted ChArUco frames required for calibration.",
    )
    parser.add_argument(
        "--min-charuco-corners",
        type=int,
        default=8,
        help="Minimum interpolated ChArUco corners required to accept a frame.",
    )
    parser.add_argument(
        "--output-dir",
        default="output",
        help="Directory for captures and calibration result.",
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


def get_aruco_dictionary(dict_name: str):
    if not hasattr(cv2.aruco, dict_name):
        raise ValueError(f"Unknown ArUco dictionary: {dict_name}")
    dict_id = getattr(cv2.aruco, dict_name)
    return cv2.aruco.getPredefinedDictionary(dict_id), int(dict_id)


def create_charuco_board(
    squares_x: int,
    squares_y: int,
    square_length: float,
    marker_length: float,
    dictionary,
):
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length,
            marker_length,
            dictionary,
        )
    return cv2.aruco.CharucoBoard_create(
        squares_x,
        squares_y,
        square_length,
        marker_length,
        dictionary,
    )


def create_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def main() -> None:
    args = parse_args()

    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco is unavailable. Install opencv-contrib-python from requirements.txt"
        )

    aruco_dict, dict_id = get_aruco_dictionary(args.dictionary)
    board = create_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_length,
        args.marker_length,
        aruco_dict,
    )
    detector_params = create_detector_parameters()

    output_dir = Path(args.output_dir)
    captures_dir = output_dir / "captures"
    captures_dir.mkdir(parents=True, exist_ok=True)

    all_charuco_corners = []
    all_charuco_ids = []
    image_size = None
    accepted = 0

    print("Controls:")
    print("  c - capture current frame if enough ChArUco corners are detected")
    print("  q - finish capture and run calibration")
    print("  esc - exit without calibration")
    print(f"Stream URL: {args.url}")
    print(f"Dictionary: {args.dictionary}")
    print(
        f"Board squares: {args.squares_x}x{args.squares_y}, "
        f"square={args.square_length}, marker={args.marker_length}"
    )

    while True:
        try:
            frame = fetch_jpeg_frame(args.url)
            image_size = (frame.shape[1], frame.shape[0])
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
                gray,
                aruco_dict,
                parameters=detector_params,
            )

            charuco_corners = None
            charuco_ids = None
            charuco_count = 0

            if marker_ids is not None and len(marker_ids) > 0:
                _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    marker_corners,
                    marker_ids,
                    gray,
                    board,
                )
                if charuco_ids is not None:
                    charuco_count = int(len(charuco_ids))

            overlay = frame.copy()
            if marker_ids is not None and len(marker_ids) > 0:
                cv2.aruco.drawDetectedMarkers(overlay, marker_corners, marker_ids)
            if charuco_ids is not None and len(charuco_ids) > 0:
                cv2.aruco.drawDetectedCornersCharuco(
                    overlay,
                    charuco_corners,
                    charuco_ids,
                    (0, 255, 0),
                )

            can_capture = charuco_count >= args.min_charuco_corners
            status_text = (
                f"ChArUco corners: {charuco_count} (ready)"
                if can_capture
                else f"ChArUco corners: {charuco_count} (need >= {args.min_charuco_corners})"
            )
            status_color = (0, 255, 0) if can_capture else (0, 0, 255)

            cv2.putText(
                overlay,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                status_color,
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                overlay,
                f"Accepted frames: {accepted}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("ESP32-CAM ChArUco Calibration", overlay)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                print("Exit requested. Calibration skipped.")
                break

            if key == ord("q"):
                if accepted < args.min_frames:
                    print(
                        f"Not enough frames: {accepted} < {args.min_frames}. "
                        "Capture more frames first."
                    )
                else:
                    print("Running ChArUco calibration...")
                    retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                        all_charuco_corners,
                        all_charuco_ids,
                        board,
                        image_size,
                        None,
                        None,
                    )

                    total_error = 0.0
                    for i in range(len(all_charuco_corners)):
                        object_points, _ = board.matchImagePoints(
                            all_charuco_corners[i],
                            all_charuco_ids[i],
                        )
                        projected, _ = cv2.projectPoints(
                            object_points,
                            rvecs[i],
                            tvecs[i],
                            camera_matrix,
                            dist_coeffs,
                        )
                        error = cv2.norm(all_charuco_corners[i], projected, cv2.NORM_L2) / len(projected)
                        total_error += float(error)
                    mean_error = total_error / len(all_charuco_corners)

                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    json_path = output_dir / f"calibration_{ts}.json"
                    npz_path = output_dir / f"calibration_{ts}.npz"

                    result = {
                        "rms_reprojection_error": float(retval),
                        "mean_projection_error": float(mean_error),
                        "image_size": [int(image_size[0]), int(image_size[1])],
                        "pattern_type": "charuco",
                        "dictionary": args.dictionary,
                        "dictionary_id": dict_id,
                        "squares_x": args.squares_x,
                        "squares_y": args.squares_y,
                        "square_length": args.square_length,
                        "marker_length": args.marker_length,
                        "camera_matrix": camera_matrix.tolist(),
                        "dist_coeffs": dist_coeffs.tolist(),
                    }
                    json_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
                    np.savez(
                        npz_path,
                        camera_matrix=camera_matrix,
                        dist_coeffs=dist_coeffs,
                        image_width=image_size[0],
                        image_height=image_size[1],
                    )

                    print(f"Calibration saved: {json_path}")
                    print(f"NPZ saved: {npz_path}")
                    print(f"RMS reprojection error: {retval:.6f}")
                    print(f"Mean projection error: {mean_error:.6f}")
                    break

            if key == ord("c") and can_capture:
                all_charuco_corners.append(charuco_corners.copy())
                all_charuco_ids.append(charuco_ids.copy())

                accepted += 1
                capture_path = captures_dir / f"capture_{accepted:03d}.jpg"
                cv2.imwrite(str(capture_path), frame)
                print(
                    f"Accepted frame #{accepted}: {capture_path} "
                    f"(corners={charuco_count})"
                )

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
