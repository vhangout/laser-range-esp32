import argparse
from pathlib import Path

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a printable ChArUco board image.")
    parser.add_argument("--squares-x", type=int, default=10, help="Number of squares along X.")
    parser.add_argument("--squares-y", type=int, default=7, help="Number of squares along Y.")
    parser.add_argument(
        "--square-length-mm",
        type=float,
        default=17.0,
        help="Square side length in millimeters.",
    )
    parser.add_argument(
        "--marker-length-mm",
        type=float,
        default=12.0,
        help="Marker side length in millimeters.",
    )
    parser.add_argument(
        "--dictionary",
        default="DICT_6X6_250",
        help="ArUco dictionary name from cv2.aruco (for example DICT_6X6_250).",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="Output print DPI used to convert physical size to pixels.",
    )
    parser.add_argument(
        "--margin-mm",
        type=float,
        default=8.0,
        help="Outer white margin in millimeters.",
    )
    parser.add_argument(
        "--output",
        default="output/charuco_board.png",
        help="Output image file path.",
    )
    return parser.parse_args()


def mm_to_px(mm: float, dpi: int) -> int:
    return max(1, int(round(mm * dpi / 25.4)))


def get_aruco_dictionary(dict_name: str):
    if not hasattr(cv2.aruco, dict_name):
        raise ValueError(f"Unknown ArUco dictionary: {dict_name}")
    dict_id = getattr(cv2.aruco, dict_name)
    return cv2.aruco.getPredefinedDictionary(dict_id)


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


def render_board_image(board, width_px: int, height_px: int, margin_px: int):
    size = (width_px, height_px)
    if hasattr(board, "generateImage"):
        return board.generateImage(size, marginSize=margin_px, borderBits=1)
    return board.draw(size, marginSize=margin_px, borderBits=1)


def main() -> None:
    args = parse_args()

    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco is unavailable. Install opencv-contrib-python from requirements.txt"
        )

    dictionary = get_aruco_dictionary(args.dictionary)
    board = create_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_length_mm,
        args.marker_length_mm,
        dictionary,
    )

    board_width_mm = args.squares_x * args.square_length_mm
    board_height_mm = args.squares_y * args.square_length_mm

    width_px = mm_to_px(board_width_mm, args.dpi)
    height_px = mm_to_px(board_height_mm, args.dpi)
    margin_px = mm_to_px(args.margin_mm, args.dpi)

    board_image = render_board_image(board, width_px, height_px, margin_px)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(output_path), board_image)
    if not ok:
        raise RuntimeError(f"Failed to write board image to: {output_path}")

    print(f"Saved: {output_path}")
    print(
        f"Board: {args.squares_x}x{args.squares_y}, square={args.square_length_mm} mm, "
        f"marker={args.marker_length_mm} mm"
    )
    print(f"Dictionary: {args.dictionary}")
    print(f"Board physical size: {board_width_mm:.2f} x {board_height_mm:.2f} mm")
    print(f"Image size: {width_px} x {height_px} px at {args.dpi} DPI")


if __name__ == "__main__":
    main()
