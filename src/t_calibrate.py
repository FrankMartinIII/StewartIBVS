import numpy as np
import cv2
import glob
import os
import pickle

CHESSBOARD_SIZE = (9,6)
SQUARE_SIZE = 2.36 #Size of squares in cm
CALIBRATION_IMAGES_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_images/*.jpg'))
OUTPUT_DIRECTORY = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_params'))
SAVE_UNDISTORTED = True

#This is a new version of the calibrate function I created using chatgpt to measure the impact of cornerSubPix on reprojection error
def calibrate_camera():
    # Prepare object points
    objPoints = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objPoints[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objPoints = objPoints * SQUARE_SIZE

    objPointsStore = []
    imgPointsStore_raw = []    # Without cornerSubPix
    imgPointsStore_refined = []  # With cornerSubPix

    images = glob.glob(CALIBRATION_IMAGES_PATH)
    if not images:
        print(f"No calibration images found at {CALIBRATION_IMAGES_PATH}")
        return None, None, None, None, None

    if not os.path.exists(OUTPUT_DIRECTORY):
        os.makedirs(OUTPUT_DIRECTORY)
    valid_image_paths = []
    for index, fileName in enumerate(images):
        curImg = cv2.imread(fileName)
        greyImg = cv2.cvtColor(curImg, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(greyImg, CHESSBOARD_SIZE, None)

        if ret:
            valid_image_paths.append(fileName)
            objPointsStore.append(objPoints)
            imgPointsStore_raw.append(corners)  # Unrefined corners

            # Refine
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.0001)
            corners2 = cv2.cornerSubPix(greyImg, corners, (11,11), (-1,-1), criteria)
            imgPointsStore_refined.append(corners2)

            cv2.drawChessboardCorners(curImg, CHESSBOARD_SIZE, corners2, ret)
            output_img_path = os.path.join(OUTPUT_DIRECTORY, f'corners_{os.path.basename(fileName)}')
            cv2.imwrite(output_img_path, curImg)

            print(f"Processed image {index+1}/{len(images)}: {fileName} - Chessboard found")
        else:
            print(f"Processed image {index+1}/{len(images)}: {fileName} - Chessboard NOT found")

    if not objPointsStore:
        print("ERROR: No valid chessboard patterns detected.")
        return None, None, None, None, None

    # Calibrate with refined points
    ret_refined, K_refined, dist_refined, rvecs_refined, tvecs_refined = cv2.calibrateCamera(
        objPointsStore, imgPointsStore_refined, (greyImg.shape[1], greyImg.shape[0]), None, None)

    # Calibrate with raw corners
    ret_raw, K_raw, dist_raw, rvecs_raw, tvecs_raw = cv2.calibrateCamera(
        objPointsStore, imgPointsStore_raw, (greyImg.shape[1], greyImg.shape[0]), None, None)

    # Save refined calibration
    calibrationData = {
        'cameraMatrix': K_refined,
        'distortionCoeff': dist_refined,
        'rotationVectors': rvecs_refined,
        'translationVectors': tvecs_refined,
        'reprojectionError': ret_refined
    }

    with open(os.path.join(OUTPUT_DIRECTORY, 'calibration_data_refined.pkl'), 'wb') as f:
        pickle.dump(calibrationData, f)

    np.savetxt(os.path.join(OUTPUT_DIRECTORY, 'camera_matrix.txt'), K_refined)
    np.savetxt(os.path.join(OUTPUT_DIRECTORY, 'distortion_coefficients.txt'), dist_refined)

    print("\nReprojection Errors:")
    print("  With cornerSubPix: ", ret_refined)
    print("  Without cornerSubPix: ", ret_raw)

    # Visualize reprojection for refined
    visualize_reprojection(valid_image_paths, objPointsStore, imgPointsStore_refined,
                       rvecs_refined, tvecs_refined, K_refined, dist_refined,
                       OUTPUT_DIRECTORY, tag="refined")

    # Visualize reprojection for raw
    visualize_reprojection(valid_image_paths, objPointsStore, imgPointsStore_raw,
                       rvecs_raw, tvecs_raw, K_raw, dist_raw,
                       OUTPUT_DIRECTORY, tag="raw")


    return ret_refined, K_refined, dist_refined, rvecs_refined, tvecs_refined

def visualize_reprojection(images, objpoints, imgpoints, rvecs, tvecs, K, dist, output_dir, tag="refined"):
    for i in range(len(images)):
        img = cv2.imread(images[i])
        img = cv2.resize(img, (640, 480))  # optional resize for display
        imgpoints_proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)

        # Draw detected corners in RED
        for pt in imgpoints[i]:
            cv2.circle(img, tuple(pt.ravel().astype(int)), 5, (0,0,255), -1)

        # Draw projected points in GREEN
        for pt in imgpoints_proj:
            cv2.circle(img, tuple(pt.ravel().astype(int)), 3, (0,255,0), -1)

        out_path = os.path.join(output_dir, f"reprojection_{tag}_{i}.jpg")
        cv2.imwrite(out_path, img)

def main():
    calibrate_camera()
if __name__ == "__main__":
    main()