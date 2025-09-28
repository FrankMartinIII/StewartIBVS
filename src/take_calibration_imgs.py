import cv2
import os
import time


def capture_calibration_images_single_cam(cameraNum):
    #Capture parameters
    CAMERA_ID = cameraNum
    CHESSBOARD_SIZE = (10,7)
    OUTPUT_DIRECTORY = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_images_home_webcam', str(cameraNum)))
    #press c to capture image
    if not os.path.exists(OUTPUT_DIRECTORY):
        os.makedirs(OUTPUT_DIRECTORY)

    #Open camera
    cap = cv2.VideoCapture(CAMERA_ID)

    if not cap.isOpened():
        print("Error: could not open camera ", CAMERA_ID)
        return
    
    # Get camera resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution: {width}x{height}")

    imgCounter = 0
    while True:
        ret, frame = cap.read()
        rawFrame = frame
        if not ret:
            print("Failed to capture image")
            break

        #Convert image to greyscale
        greyImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #cv2.imshow("TEST", greyImg)
        #cv2.waitKey(0)

        #Find chessboard corners
        ret_chess, corners = cv2.findChessboardCorners(greyImg, CHESSBOARD_SIZE, None)

        #Draw corners if found
        if ret_chess:
            rawFrame = frame.copy()
            cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, corners, ret_chess)
            cv2.putText(frame, "Chessboard detected!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        print("Image ", imgCounter, " captured")
        cv2.imshow("Camera calibration", frame)

        key = cv2.waitKey(1) & 0xFF

        # 'q' or Escape to quit
        if key == ord('q') or key == 27:  # 27 is the ASCII code for Escape
            print("Exiting...")
            break
        
        # 'c' to capture
        elif key == ord('c'):
            # Save the image
            img_name = os.path.join(OUTPUT_DIRECTORY, f"calibration_{imgCounter:02d}.jpg")
            cv2.imwrite(img_name, rawFrame)
            print(f"Captured {img_name}")
            
            imgCounter += 1

    #end while
    cap.release()
    cv2.destroyAllWindows()

    print("Captured ", imgCounter, " images")


if __name__ == "__main__":
    capture_calibration_images_single_cam(0)
    #capture_calibration_images_single_cam(2)
