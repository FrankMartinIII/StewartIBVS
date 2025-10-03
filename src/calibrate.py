import numpy as np
import cv2
import glob
import os
import pickle



def calibrate_camera(cameraNum):
    '''
    Returns
    ret: RMS reprojection error
    K: camera matrix
    distCoef: distorition coefficient
    rotVecs: rotation vectors (omega)
    transVecs: translation vectors
    '''
    CHESSBOARD_SIZE = (6,5)
    #SQUARE_SIZE = 2.36 #Size of squares in cm
    SQUARE_SIZE = 20
    folderStr = 'calibration_images_home_webcam/' + str(cameraNum) + '/*.jpg'
    #CALIBRATION_IMAGES_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_images/*.jpg'))
    CALIBRATION_IMAGES_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', folderStr))
    #CALIBRATION_IMAGES_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'FrankCalib/frames/J2/*.png'))
    OUTPUT_DIRECTORY = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_params_home_webcam', str(cameraNum)))
    SAVE_UNDISTORTED = True

    if cameraNum == 5:
        CALIBRATION_IMAGES_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'FrankCalib/frames/D2/*.png'))
        CHESSBOARD_SIZE = (7,4)
    #if cameraNum == 6:
    #    CALIBRATION_IMAGES_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'FrankCalib/frames/J2/*.png'))
    #    CHESSBOARD_SIZE = (7,4)
    if cameraNum == 7:
        CALIBRATION_IMAGES_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_images/kaggle0/*.png'))
        CHESSBOARD_SIZE = (11,7)
    if cameraNum == 8:
        CALIBRATION_IMAGES_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_images/kaggle0/*.png'))
        CHESSBOARD_SIZE = (11,7)
    # Prepare object points (0,0,0), (1,0,0), (2,0,0) ... (8,5,0)
    objPoints = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objPoints[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

    #Scale object points by square size for real-world measurements
    objPoints = objPoints * SQUARE_SIZE

    #Create arrays to store object points and image points from all images
    objPointsStore = []
    imgPointsStore = []

    #Get list of calibration images
    print("Checking path: ", CALIBRATION_IMAGES_PATH)
    images = glob.glob(CALIBRATION_IMAGES_PATH)
    if not images:
        print(f"No calibration images found at {CALIBRATION_IMAGES_PATH}")
        return None, None, None, None, None
    
    #Create output directory
    if not os.path.exists(OUTPUT_DIRECTORY):
        os.makedirs(OUTPUT_DIRECTORY)

    print("Images found: ", len(images))
    #for n in images:
    #    print(n)

    for index, fileName in enumerate(images):
        curImg = cv2.imread(fileName)
        greyImg = cv2.cvtColor(curImg, cv2.COLOR_BGR2GRAY)

        cv2.imshow("img", greyImg)
        cv2.waitKey(0)
        #Find chessboard corners
        ret, corners = cv2.findChessboardCorners(greyImg, CHESSBOARD_SIZE, None)

        #If found, add object points and image points
        if ret:
            objPointsStore.append(objPoints)

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            #criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            corners2 = cv2.cornerSubPix(greyImg, corners, (11,11), (-1,-1), criteria)
            imgPointsStore.append(corners2)

            #Draw and display corners
            cv2.drawChessboardCorners(curImg, CHESSBOARD_SIZE, corners2, ret)

            # Save image with corners drawn
            output_img_path = os.path.join(OUTPUT_DIRECTORY, f'corners_{os.path.basename(fileName)}')
            cv2.imwrite(output_img_path, curImg)

            print(f"Processed image {index}/{len(images)}: {fileName} - Chessboard found")
        else:
            print(f"Processed image {index}/{len(images)}: {fileName} - Chessboard NOT found")

    if not objPointsStore:
        print("ERROR No chessboards detected in any images")
        return None, None, None, None, None
    
    #Calibrate camera, K is the intrinsic camera matrix
    ret, K, distCoef, rotVecs, transVecs = cv2.calibrateCamera(objPointsStore, imgPointsStore, (greyImg.shape[1], greyImg.shape[0]), None, None)
    print("Distortion coefficients", distCoef)

    #Save calibration data
    calibrationData = {
        'cameraMatrix' : K,
        'distortionCoeff': distCoef,
        'rotationVectors': rotVecs,
        'translationVectors': transVecs,
        'reprojectionError': ret
    }
    
    with open(os.path.join(OUTPUT_DIRECTORY, 'calibration_data.pkl'), 'wb') as f:
        pickle.dump(calibrationData, f)

    #Save camera matrix and distortion coefficient as text files
    np.savetxt(os.path.join(OUTPUT_DIRECTORY, 'camera_matrix.txt'), K)
    np.savetxt(os.path.join(OUTPUT_DIRECTORY, 'distortion_coefficients.txt'), distCoef)
    print("Reprojection error: ", ret)
    
    return ret, K, distCoef, rotVecs, transVecs

def main():
    calibrate_camera(4)
if __name__ == "__main__":
    main()
