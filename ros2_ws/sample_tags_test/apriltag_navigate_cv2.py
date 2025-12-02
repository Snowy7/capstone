#!/usr/bin/env python3
import cv2
import numpy as np

# ----- CONFIG -----
CAMERA_ID = 0
CAMERA_PARAMS_FILE = "camera_params.npz"

FAMILY = "tag36h11"
TARGET_TAG_ID = 0          # must match printed tag
TAG_SIZE = 0.029            # meters, set from your print measurement

TARGET_DIST = 0.5          # desired distance to tag along z (m)
LINEAR_KP = 0.8
ANGULAR_KP = 2.0
# -------------------

FAMILY_TO_DICT = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

def load_camera_params(path):
    data = np.load(path)
    return data["camera_matrix"], data["dist_coeffs"]

def main():
    if FAMILY not in FAMILY_TO_DICT:
        print(f"Unsupported family {FAMILY} for cv2.aruco.")
        return

    try:
        camera_matrix, dist_coeffs = load_camera_params(CAMERA_PARAMS_FILE)
        print("Loaded camera parameters.")
    except Exception as e:
        print(f"Error loading camera params: {e}")
        return

    aruco_dict = cv2.aruco.getPredefinedDictionary(FAMILY_TO_DICT[FAMILY])
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return

    print("Press 'q' to quit.")
    print(f"Looking for {FAMILY} tag with ID={TARGET_TAG_ID}")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            # Flatten ids array for convenience
            ids_flat = ids.flatten()

            if TARGET_TAG_ID in ids_flat:
                idx = list(ids_flat).index(TARGET_TAG_ID)
                tag_corners = [corners[idx]]

                # Estimate pose of this marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    tag_corners, TAG_SIZE, camera_matrix, dist_coeffs
                )

                rvec = rvecs[0][0]
                tvec = tvecs[0][0]
                x, y, z = tvec  # camera-frame: x right, y down, z forward (approx)

                # Control:
                dist_error = z - TARGET_DIST
                yaw_error = x  # use x offset as proxy for yaw misalignment

                linear_cmd = -LINEAR_KP * dist_error
                angular_cmd = -ANGULAR_KP * yaw_error

                linear_cmd = float(np.clip(linear_cmd, -0.5, 0.5))
                angular_cmd = float(np.clip(angular_cmd, -1.0, 1.0))

                # Draw detection and axes
                cv2.aruco.drawDetectedMarkers(frame, tag_corners,
                                              np.array([[TARGET_TAG_ID]]))
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                  rvec, tvec, TAG_SIZE * 0.5)

                text = (
                    f"ID:{TARGET_TAG_ID} "
                    f"x:{x:.3f} y:{y:.3f} z:{z:.3f} "
                    f"v:{linear_cmd:.2f} w:{angular_cmd:.2f}"
                )
                cv2.putText(frame, text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 255), 2)

                # These v,w are what you'd send to your robot.
                # For ROS2: publish as geometry_msgs/Twist.
            else:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.putText(frame,
                            f"Tag {TARGET_TAG_ID} not in view",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255), 2)
        else:
            cv2.putText(frame, "No AprilTags detected",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2)

        cv2.imshow("AprilTag Navigation", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()