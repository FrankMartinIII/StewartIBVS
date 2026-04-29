import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import PoseStamped
from vs_stewart.msg import PoseData
from collections import deque
import os
import pickle
from scipy.spatial.transform import Rotation

class Tracker:
    def __init__(self, camera_matrix, distortion_coefficients, T_cam_to_world, T_world_to_camera, camera_index=0, charuco_square_size=16, charuco_marker_size=12):
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coefficients
        self.T_cam_to_world = T_cam_to_world
        #Need to correct T_cam_to_world to align with robot frame
        '''
        correction_T = np.eye(4)
        correction_T[0,:3] = [0,1,0]
        correction_T[1,:3] = [-1,0,0]
        correction_T[2,:3] = [0,0,-1]
        self.T_cam_to_world = correction_T @ self.T_cam_to_world
        self.T_world_to_camera = T_world_to_camera
        '''
        #Attempt to rotate world frame to align with robot base frame
        yaw_offset_rotation = -60 #The world frame needs to be rotated 30 degrees to align with the robot frame
        align_T = np.eye(4)
        align_T[:3,:3] = Rotation.from_euler('z', yaw_offset_rotation, degrees=True).as_matrix()
        self.T_cam_to_world = align_T @ self.T_cam_to_world

        self.T_world_to_camera = np.linalg.inv(self.T_cam_to_world)
        self.pub = rospy.Publisher("vs_stewart/charuco_vecs", PoseData, queue_size=10)
         # Open camera
        #self.cap = cv2.VideoCapture(camera_index)
        #if not self.cap.isOpened():
            #rospy.logerr("Cannot open camera")
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        #self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        #self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #rospy.loginfo(f"Camera0 resolution: {self.width}x{self.height}")
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ret, frame = self.cap.read()
        print("Frame shape:", frame.shape)
        print("Actual width:", self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        print("Actual height:", self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        #This node will use a charuco board for more stability
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.charuco_square_size = charuco_square_size
        self.charuco_marker_size = charuco_marker_size
        
        CHARUCO_SQUARES_DIM_X = 3
        CHARUCO_SQUARES_DIM_Y = 3
        CHARUCO_SQUARE_SIZE = charuco_square_size #16mm
        CHARUCO_MARKER_SIZE = charuco_marker_size

        try:
            #OpenCV >= 4.7
            self.parameters = cv2.aruco.DetectorParameters()
            self.charuco_board = cv2.aruco.CharucoBoard((CHARUCO_SQUARES_DIM_X, CHARUCO_SQUARES_DIM_Y), CHARUCO_SQUARE_SIZE, CHARUCO_MARKER_SIZE, self.aruco_dict)
        except:
            #OpenCV < 4.7
            self.parameters = cv2.aruco.DetectorParameters_create()
            self.charuco_board = cv2.aruco.CharucoBoard_create(CHARUCO_SQUARES_DIM_X, CHARUCO_SQUARES_DIM_Y, CHARUCO_SQUARE_SIZE, CHARUCO_MARKER_SIZE, self.aruco_dict)
            

        # Buffers
        self.pose_buffer = deque(maxlen=10)   # rolling buffer of last detections
        self.initial_T = None                 # initial transform (4x4)
        self.initial_charuco_in_world_position_vector = [0, 0, 0]
        self.initial_charuco_in_world_orientation_vector = [0, 0, 0, 1]

        #Check matrix and frame size
        h, w = frame.shape[:2]
        print("frame.shape:", frame.shape)
        print("cameraMatrix:\n", camera_matrix)
        fx = camera_matrix[0,0]; fy = camera_matrix[1,1]
        cx = camera_matrix[0,2]; cy = camera_matrix[1,2]
        print(f"fx,fy = {fx:.3f}, {fy:.3f}")
        print(f"cx,cy = {cx:.3f}, {cy:.3f} (image center would be {(w/2):.1f},{(h/2):.1f})")
        print("cameraMatrix shape:", camera_matrix.shape)



        


    def create_transformation_matrix(self, tvec, rvec):
        R, _ = cv2.Rodrigues(rvec)

        #Define identity 4x4 matrix to start
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = tvec.flatten()
        return T
    
    def set_initial_pose(self):
        #rospy.loginfo(f"Pose bufffer: {self.pose_buffer}")
        translations = np.array(T[0:3,3] for T in self.pose_buffer)
        translations = np.array([T[0:3, 3] for T in self.pose_buffer])
        mean_t = np.mean(translations, axis=0)

        #Average rotations
        R_sum = np.zeros((3,3))
        for T in self.pose_buffer:
            R_sum += T[0:3, 0:3]

        # Project back to nearest rotation matrix using SVD
        U, _, Vt = np.linalg.svd(R_sum)
        R_avg = U @ Vt

        # Build full transform
        T_avg = np.eye(4)
        T_avg[0:3, 0:3] = R_avg
        T_avg[0:3, 3] = mean_t

        self.initial_T = T_avg

        r = Rotation.from_matrix(R_avg)
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        rospy.loginfo(f"Initial pose X: {T_avg[0,3]} Y: {T_avg[1,3]} Z: {T_avg[2,3]}")
        rospy.loginfo(f"Roll: {roll} Pitch: {pitch} Yaw: {yaw}")
        rospy.loginfo("Initial pose set.")
    
    def to_pose_stamped(self, T):
        #Convert transformation matrix to pose with rpy in degrees
        #This probably is not what I need anymore. Making a new function to just create a pose from xyzrpy
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "platform_measured"

        pose_msg.pose.position.x = T[0,3]
        pose_msg.pose.position.y = T[1,3]
        pose_msg.pose.position.z = T[2,3]

        Rmat = T[:3,:3]
        r = Rotation.from_matrix(Rmat)
        roll,pitch,yaw = r.as_euler('xyz', degrees=True)

        pose_msg.pose.orientation.x = roll
        pose_msg.pose.orientation.y = pitch
        pose_msg.pose.orientation.z = yaw
        pose_msg.pose.orientation.w = 1.0  # not a true quaternion here

        return pose_msg
    
    def make_pose_stamped(self, x_trans, y_trans, z_trans, r, p, y):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "platform_measured"

        pose_msg.pose.position.x = x_trans
        pose_msg.pose.position.y = y_trans
        pose_msg.pose.position.z = z_trans

        pose_msg.pose.orientation.x = r
        pose_msg.pose.orientation.y = p
        pose_msg.pose.orientation.z = y
        pose_msg.pose.orientation.w = 1.0  # not a true quaternion here

        return pose_msg

    def draw_axes_on_image(self, img, T, camera_matrix, dist_coeffs, label="Frame", axis_length=50):
        """
        Draw a 3D axes for a given 4x4 transform T on an image.
        
        Args:
            img: OpenCV image
            T: 4x4 homogeneous transform (object -> camera frame)
            camera_matrix: Intrinsics
            dist_coeffs: Distortion coefficients
            label: Text label for the frame origin
            axis_length: Length of each axis in mm
        """
        R = T[:3, :3]
        t = T[:3, 3]

        # Make sure the origin is in front of the camera (positive Z)
        if t[2] <= 0:
            t[2] = 50.0  # minimum positive Z in mm

        # Define axes in local frame
        axes_3D = np.float32([
            [0, 0, 0],                  # origin
            [axis_length, 0, 0],        # X
            [0, axis_length, 0],        # Y
            [0, 0, axis_length]         # Z
        ])

        # Transform axes to camera frame
        axes_in_cam = (R @ axes_3D.T).T + t

        # Project points
        imgpts, _ = cv2.projectPoints(axes_in_cam, np.zeros(3), np.zeros(3), camera_matrix, dist_coeffs)
        imgpts = np.clip(imgpts.squeeze(), 0, [img.shape[1]-1, img.shape[0]-1])  # keep in image bounds
        origin = tuple(imgpts[0].astype(int))

        # Draw axes lines
        cv2.line(img, origin, tuple(imgpts[1].astype(int)), (0,0,255), 3)  # X - Red
        cv2.line(img, origin, tuple(imgpts[2].astype(int)), (0,255,0), 3)  # Y - Green
        cv2.line(img, origin, tuple(imgpts[3].astype(int)), (255,0,0), 3)  # Z - Blue

        # Draw label
        cv2.putText(img, label, (origin[0]+5, origin[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        return img

    def run(self):
        rate = rospy.Rate(30)  # adjust publish rate

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to grab frame")
                continue
            #preview_frame = cv2.resize(frame, (960, 540))
            #This aruco detection is the same as previously
            cv2.imshow("Frame", frame)
            grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(grey, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None:
                #Draw the markers on the image
                frame_with_aruco = frame.copy()
                # Draw camera frame at origin (0,0,100 mm in front)
                self.draw_axes_on_image(frame_with_aruco, np.eye(4), self.camera_matrix, self.distortion_coeffs, label="Camera Frame")
                # Draw world frame
                self.draw_axes_on_image(frame_with_aruco, self.T_world_to_camera, self.camera_matrix, self.distortion_coeffs, label="World Frame")
                cv2.aruco.drawDetectedMarkers(frame_with_aruco, corners)

                #Instead of estimatePoseSingleMarker, using interpolateCornersCharuco. 
                # receives the detected markers and returns the 2D position of the chessboard corners from a ChArUco board using the detected Aruco markers. 
                # If camera parameters are provided, the process is based in an approximated pose estimation
                #Returns the number of interpolated corners, the interpolated corners, and the corner identifiers
                num_charuco_corners, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, grey, self.charuco_board, cameraMatrix= self.camera_matrix, distCoeffs=self.distortion_coeffs)
                #If at least one corner is detected
                if num_charuco_corners > 0:
                    cv2.aruco.drawDetectedCornersCharuco(frame_with_aruco, charuco_corners, charuco_ids, (255,244,244))
                    #rvec: cv2.typing.MatLike = np.zeros((3, 3, 1), dtype=np.uint8)
                    #tvec: cv2.typing.MatLike = np.zeros((3, 3, 1), dtype=np.uint8)
                    valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids, self.charuco_board, self.camera_matrix, self.distortion_coeffs, None, None)
                    if valid:

                        #By default, estimatePoseCharucoBoard gives matrix with center at bottom left of the board.
                        #This code moves the center to the center of the board (hopefully)
                        T_current_in_camera = self.create_transformation_matrix(tvec, rvec)
                        shift_bottomleft_to_center = np.eye(4)
                        #Shift amount is half the total board width
                        shift_amount = self.charuco_square_size * 3 / 2
                        shift_bottomleft_to_center[0,3] = shift_amount
                        shift_bottomleft_to_center[1,3] = shift_amount
                        shift_bottomleft_to_center[2,3] = -13
                        T_current_in_camera = T_current_in_camera @ shift_bottomleft_to_center
                        
                        R_centered = T_current_in_camera[:3,:3]
                        rvec_centered, _ = cv2.Rodrigues(R_centered)
                        rvec = rvec_centered.flatten()
                        tvec_centered = T_current_in_camera[:3,3]
                        tvec = tvec_centered.flatten()


                        cv2.drawFrameAxes(frame_with_aruco, self.camera_matrix, self.distortion_coeffs, rvec, tvec, 80)
                        print("RVEC")
                        print(rvec)
                        print("TVEC")
                        print(tvec)

                        #Publish
                        pose_msg = PoseData()
                        pose_msg.header.stamp = rospy.Time.now()
                        pose_msg.charucoRvec.x = rvec[0]
                        pose_msg.charucoRvec.y = rvec[1]
                        pose_msg.charucoRvec.z = rvec[2]
                        pose_msg.charucoTvec.x = tvec[0]
                        pose_msg.charucoTvec.y = tvec[1]
                        pose_msg.charucoTvec.z = tvec[2]


                        print(rvec.shape)
                        
                        T_current_in_world = self.T_cam_to_world @ T_current_in_camera

                        if self.initial_T is None:
                            #Buffer of poses for averaging initial pose
                            #Make sure pose it in world frame before stored
                            self.pose_buffer.append(T_current_in_world)
                        else:
                            #Get xyz diff
                            delta_pos_world = T_current_in_world[:3,3] - self.initial_T[:3,3]

                            #Rotational diff
                            R_initial_world = self.initial_T[:3,:3]
                            R_current_world = T_current_in_world[:3,:3]
                            R_delta = R_current_world @ R_initial_world.T
                            
                            #Convert to Euler
                            r_delta = Rotation.from_matrix(R_delta)
                            roll_rel, pitch_rel, yaw_rel = r_delta.as_euler('xyz', degrees=True)

                            #Text display
                            text_pos = f"Rel Pos (mm) X: {delta_pos_world[0]:.1f} Y: {delta_pos_world[1]:.1f} Z: {delta_pos_world[2]:.1f}"
                            text_rot = f"Rel Rot (deg) R: {roll_rel:.2f} P: {pitch_rel:.2f} Y: {yaw_rel:.2f}"

                            cv2.putText(frame_with_aruco, text_pos, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                            cv2.putText(frame_with_aruco, text_rot, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                            #Current pose in world coordinated pushed into msg
                            pose_msg.charucoInWorldCoordinates.position.x = delta_pos_world[0]
                            pose_msg.charucoInWorldCoordinates.position.y = delta_pos_world[1]
                            pose_msg.charucoInWorldCoordinates.position.z = delta_pos_world[2]
                            pose_msg.charucoInWorldCoordinates.orientation.x = roll_rel
                            pose_msg.charucoInWorldCoordinates.orientation.y = pitch_rel
                            pose_msg.charucoInWorldCoordinates.orientation.z = yaw_rel
                            pose_msg.charucoInWorldCoordinates.orientation.w = 1.0

                            #initial_T also stored in msg
                            pose_msg.charucoStartingInWorldCoordinates.position.x = self.initial_charuco_in_world_position_vector[0]
                            pose_msg.charucoStartingInWorldCoordinates.position.y = self.initial_charuco_in_world_position_vector[1]
                            pose_msg.charucoStartingInWorldCoordinates.position.z = self.initial_charuco_in_world_position_vector[2]
                            pose_msg.charucoStartingInWorldCoordinates.orientation.x = self.initial_charuco_in_world_orientation_vector[0]
                            pose_msg.charucoStartingInWorldCoordinates.orientation.y = self.initial_charuco_in_world_orientation_vector[1]
                            pose_msg.charucoStartingInWorldCoordinates.orientation.z = self.initial_charuco_in_world_orientation_vector[2]
                        self.pub.publish(pose_msg)



                    else:
                        print("invalid pose")
                
                #preview_frame_with_aruco = cv2.resize(frame_with_aruco, (960, 540))
                else:
                    print("num_charuco_corners <= 0")
                cv2.imshow("Aruco detection", frame_with_aruco)


            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                if len(self.pose_buffer) == self.pose_buffer.maxlen:
                    self.set_initial_pose()
                    #Taking this initial position and rotation of charuco and storing it for use in message publishing
                    #XYZ position
                    pos_world = self.initial_T[:3,3]
                    self.initial_charuco_in_world_position_vector[0] = pos_world[0]
                    self.initial_charuco_in_world_position_vector[1] = pos_world[1]
                    self.initial_charuco_in_world_position_vector[2] = pos_world[2]
                    #RPY orientation
                    R_world = self.initial_T[:3,:3]
                    r_world = Rotation.from_matrix(R_world)
                    roll, pitch, yaw = r_world.as_euler('xyz', degrees=True)
                    self.initial_charuco_in_world_orientation_vector[0] = roll
                    self.initial_charuco_in_world_orientation_vector[1] = pitch
                    self.initial_charuco_in_world_orientation_vector[2] = yaw
                    self.initial_charuco_in_world_orientation_vector[3] = 1.0

        
        
def read_camera_params(filePath):
    with open(filePath, 'rb') as f:
        data = pickle.load(f)
    cameraMatrix1 = data['cameraMatrix']
    distCoeffs1 = data['distortionCoeff']
    rotation_vecs = data['rotationVectors']
    translation_vecs = data['translationVectors']
    reprojection_err = data['reprojectionError']
    return cameraMatrix1, distCoeffs1

def read_robotframe_calibration(filePath):
    with open(filePath, 'rb') as f:
        data = pickle.load(f)
    T_camera_to_world = data['T_camera_to_world']
    T_world_to_camera = data['T_world_to_camera']
    return T_camera_to_world, T_world_to_camera

if __name__ == "__main__":
    rospy.init_node("pose_tracking_node2")
    #Read in calibration parameters for the cameras
    CAMERA_CALIB_PARAM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_params_home_webcam/4', 'calibration_data.pkl'))
    cameraMat1, distCoeffs1 = read_camera_params(CAMERA_CALIB_PARAM_DIR)
    FRAME_PARAM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_params_home_webcam/RobotBaseToCam/TransformationMatricesForStewCalib.pkl'))
    T_camera_to_world, T_world_to_camera = read_robotframe_calibration(FRAME_PARAM_DIR)
    tracker = Tracker(cameraMat1, distCoeffs1, T_camera_to_world, T_world_to_camera, camera_index=4)
    tracker.run()
    