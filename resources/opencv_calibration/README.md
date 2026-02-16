# ESP32-CAM OpenCV Calibration (ChArUco)

Calibration helper for ESP32-CAM stream endpoint:

`http://192.168.1.73/rawcam`

## 1. Setup

```powershell
cd resources/opencv_calibration
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## 2. Prepare ChArUco board

Use board type `ChArUco` in the generator.

Recommended values for current defaults:

- `squaresX = 10`
- `squaresY = 7`
- `squareLength = 17 mm`
- `markerLength = 12 mm`
- dictionary `DICT_6X6_250`

Print with exact scale (`100%`, no fit-to-page).

You can also generate board locally:

```powershell
python generate_charuco_board.py --output output/charuco_board.png
```

Or with explicit parameters:

```powershell
python generate_charuco_board.py --squares-x 10 --squares-y 7 --square-length-mm 17 --marker-length-mm 12 --dictionary DICT_6X6_250 --dpi 300 --output output/charuco_board_10x7.png
```

## 3. Run calibration capture

```powershell
python calibrate_from_stream.py --url http://192.168.1.73/rawcam
```

Controls:

- `c`: capture frame if enough ChArUco corners are detected
- `q`: run calibration (requires enough frames)
- `esc`: exit without calibration

Recommended:

- Capture at least 12 to 25 frames.
- Change board angle and position between captures.
- Include center and edges of frame.

Outputs:

- `output/captures/*.jpg`
- `output/calibration_YYYYMMDD_HHMMSS.json`
- `output/calibration_YYYYMMDD_HHMMSS.npz`

## 4. Preview undistortion

```powershell
python preview_undistort.py --url http://192.168.1.73/rawcam --calibration output/calibration_YYYYMMDD_HHMMSS.json
```

Controls:

- `q` or `esc`: exit

## Optional parameters

```powershell
python calibrate_from_stream.py --help
```

Main options:

- `--squares-x`, `--squares-y` for ChArUco board size (defaults `10x7`)
- `--square-length` board square size in physical units (default `17.0`)
- `--marker-length` marker size in same units (default `12.0`)
- `--dictionary` ArUco dictionary name (default `DICT_6X6_250`)
- `--min-charuco-corners` minimum detected ChArUco corners per accepted frame (default `8`)
- `--min-frames` minimum captured frames before calibration (default `12`)
- `--output-dir` output folder (default `output`)

Generator script options:

- `--square-length-mm`, `--marker-length-mm` board sizes in mm
- `--dpi` output DPI for print-ready image conversion
- `--margin-mm` white border around board
- `--output` output PNG path
