import cv2
import numpy as np
import time

# ----- CONFIG -----
CAMERA_ID = 1  # usually 0; change if needed
CHESSBOARD_SIZE = (8, 5)  # inner corners (cols, rows)
SQUARE_SIZE = 0.03  # meters (e.g. 30mm squares)
NUM_SAMPLES = 25     # how many good views to capture
OUTPUT_FILE = "camera_params.npz"
# -------------------

def main():
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return

    # prepare object points for one chessboard view
    # (0,0,0), (1,0,0), ... scaled by SQUARE_SIZE
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    print("Press SPACE to capture a view when corners are detected.")
    print(f"Move the board around (angles, distances). Need {NUM_SAMPLES} samples.")
    print("Press 'q' to quit without saving.")

    captured = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, CHESSBOARD_SIZE, corners, found)
            cv2.putText(display, "Corners found - press SPACE to capture",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Show calibration chessboard",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, f"Captured: {captured}/{NUM_SAMPLES}",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(10) & 0xFF

        if key == ord('q'):
            print("Aborted.")
            break
        elif key == 32 and found:  # SPACE
            # refine corner locations
            corners_sub = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30, 0.001
                )
            )
            objpoints.append(objp)
            imgpoints.append(corners_sub)
            captured += 1
            print(f"Captured sample {captured}/{NUM_SAMPLES}")

            time.sleep(0.5)  # small delay so you can move the board

            if captured >= NUM_SAMPLES:
                print("Enough samples collected. Calibrating...")
                ret_calib, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                    objpoints, imgpoints, gray.shape[::-1], None, None
                )

                if not ret_calib:
                    print("Calibration failed.")
                else:
                    print("Calibration successful.")
                    print("Camera matrix:")
                    print(camera_matrix)
                    print("Distortion coefficients:")
                    print(dist_coeffs.ravel())

                    np.savez(OUTPUT_FILE,
                             camera_matrix=camera_matrix,
                             dist_coeffs=dist_coeffs)
                    print(f"Saved to {OUTPUT_FILE}")
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()