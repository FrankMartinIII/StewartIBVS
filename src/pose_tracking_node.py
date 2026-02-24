import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import PoseStamped
from collections import deque
import os
import pickle
from scipy.spatial.transform import Rotation

class Tracker:
    def __init__(self, camera_matrix, distortion_coefficients, T_cam_to_world, T_world_to_camera, camera_index=0, aruco_size=50):
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coefficients
        self.T_cam_to_world = T_cam_to_world
        self.T_world_to_camera = T_world_to_camera
        self.aruco_size = aruco_size
        self.pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=10)
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

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        #self.parameters = cv2.aruco.DetectorParameters()
        try:
            # OpenCV ≥ 4.7
            self.parameters = cv2.aruco.DetectorParameters()
        except AttributeError:
            # OpenCV < 4.7
            self.parameters = cv2.aruco.DetectorParameters_create()

        # Buffers
        self.pose_buffer = deque(maxlen=10)   # rolling buffer of last detections
        self.initial_T = None                 # initial transform (4x4)

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
            cv2.imshow("Frame", frame)
            grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(grey, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None:
                frame_with_aruco = frame.copy()
                # Draw camera frame at origin (0,0,100 mm in front)
                self.draw_axes_on_image(frame_with_aruco, np.eye(4), self.camera_matrix, self.distortion_coeffs, label="Camera Frame")

                # Draw world frame
                self.draw_axes_on_image(frame_with_aruco, self.T_world_to_camera, self.camera_matrix, self.distortion_coeffs, label="World Frame")

                #estimatePose returns tvec that describes transformation from aruco center to camera optical center, it does not return coordinates of aruco in cam frame
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.camera_matrix, self.distortion_coeffs)
                cv2.aruco.drawDetectedMarkers(frame_with_aruco, corners)
                for rvec, tvec in zip(rvecs, tvecs):
                    T_current = self.create_transformation_matrix(tvec, rvec)
                    if self.initial_T is None:
                        #Buffer of poses for averaging initial pose
                        #Make sure pose it in world frame before stored
                        T_current_in_world = self.T_cam_to_world @ T_current
                        self.pose_buffer.append(T_current_in_world)
                    else:
                        '''
                        T_TEMP = self.T_cam_to_world @ T_current
                        T_relative = np.linalg.inv(self.initial_T) @ T_current
                        R_current = T_current[:3,:3]
                        r_current = Rotation.from_matrix(R_current)
                        R_relative = T_relative[:3,:3]
                        r_relative = Rotation.from_matrix(R_relative)
                        roll_cam,pitch_cam,yaw_cam = r_current.as_euler('xyz', degrees=True)
                        roll_rel,pitch_rel,yaw_rel = r_relative.as_euler('xyz', degrees=True)
                        text_cam = f"CameraFrame X: {T_current[0,3]:.3f} mm  Y: {T_current[1,3]:.3f} mm  Z: {T_current[2,3]:.3f} mm"
                        #text_cam = f"CameraFrame X: {T_TEMP[0,3]:.3f} mm  Y: {T_TEMP[1,3]:.3f} mm  Z: {T_TEMP[2,3]:.3f} mm"
                        text_cam_rot = f"Roll: {roll_cam:.3f} Pitch: {pitch_cam:.3f}, Yaw: {yaw_cam:.3f}"
                        #text_world = f"World FrameX: {marker_world_pos[0]:.3f} mm  Y: {marker_world_pos[1]:.3f} mm  Z: {marker_world_pos[2]:.3f} mm"
                        text_world = f"Relative to original X: {T_relative[0,3]:.3f} mm  Y: {T_relative[1,3]:.3f} mm  Z: {T_relative[2,3]:.3f} mm"
                        text_world_rot = f"Roll: {roll_rel:.3f} Pitch: {pitch_rel:.3f}, Yaw: {yaw_rel:.3f}"
                        cv2.putText(frame_with_aruco, text_cam, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 2)
                        cv2.putText(frame_with_aruco, text_cam_rot, (10, self.height - 150), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 2)
                        cv2.putText(frame_with_aruco, text_world, (10, self.height - 75), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 150), 2)
                        cv2.putText(frame_with_aruco, text_world_rot, (10, self.height - 30), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 150), 2)
                        '''
                        #Starting pose of aruco in camera frame
                        T_current_aruco_in_camera_frame = T_current
                        R_current = T_current_aruco_in_camera_frame[:3,:3]
                        r_current = Rotation.from_matrix(R_current)
                        roll_aruco_cam,pitch_aruco_cam,yaw_aruco_cam = r_current.as_euler('xyz', degrees=True)
                        text_aruco_translation_camera_space = f"Aruco in camera frame X: {T_current_aruco_in_camera_frame[0,3]:.3f}, Y: {T_current_aruco_in_camera_frame[1,3]:.3f}, Z: {T_current_aruco_in_camera_frame[2,3]:.3f}"
                        text_aruco_rotation_camera_space = f"Aruco in camera frame Roll: {roll_aruco_cam:.3f} Pitch: {pitch_aruco_cam:.3f}, Yaw: {yaw_aruco_cam:.3f}"
                        cv2.putText(frame_with_aruco, text_aruco_translation_camera_space, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                        cv2.putText(frame_with_aruco, text_aruco_rotation_camera_space, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                        #Pose of aruco in world frame
                        T_current_aruco_in_world_frame = self.T_cam_to_world @ T_current_aruco_in_camera_frame #This is T_camera_to_world x T_camera_to_marker (from estimatePoseSIngleMarker)
                        R_aruco_in_world = T_current_aruco_in_world_frame[:3,:3]
                        r_aruco_in_world = Rotation.from_matrix(R_aruco_in_world)
                        roll_aruco_world,pitch_aruco_world,yaw_aruco_world = r_aruco_in_world.as_euler('xyz', degrees=True)
                        text_aruco_translation_world_space = f"Aruco in world frame X: {T_current_aruco_in_world_frame[0,3]:.3f}, Y: {T_current_aruco_in_world_frame[1,3]:.3f}, Z: {T_current_aruco_in_world_frame[2,3]:.3f}"
                        text_aruco_rotation_world_space = f"Aruco in world frame Roll: {roll_aruco_world:.3f} Pitch: {pitch_aruco_world:.3f}, Yaw: {yaw_aruco_world:.3f}"
                        cv2.putText(frame_with_aruco, text_aruco_translation_world_space, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                        cv2.putText(frame_with_aruco, text_aruco_rotation_world_space, (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                        '''
                        #Pose of the aruco in world frame relative to where it started
                        T_aruco_in_world_relative_to_start = np.linalg.inv(self.initial_T) @ T_current_aruco_in_world_frame
                        R_aruco_in_world_relative_to_start = T_aruco_in_world_relative_to_start[:3,:3]
                        r_aruco_in_world_relative_to_start = Rotation.from_matrix(R_aruco_in_world_relative_to_start)
                        roll_aruco_rel,pitch_aruco_rel,yaw_aruco_rel = r_aruco_in_world_relative_to_start.as_euler('xyz', degrees=True)
                        text_aruco_translation_rel = f"Aruco relative to start X: {T_aruco_in_world_relative_to_start[0,3]:.3f}, Y: {T_aruco_in_world_relative_to_start[1,3]:.3f}, Z: {T_aruco_in_world_relative_to_start[2,3]:.3f}"
                        text_aruco_rotation_rel = f"Aruco irelative to start Roll: {roll_aruco_rel:.3f} Pitch: {pitch_aruco_rel:.3f}, Yaw: {yaw_aruco_rel:.3f}"
                        cv2.putText(frame_with_aruco, text_aruco_translation_rel, (10, 500), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                        cv2.putText(frame_with_aruco, text_aruco_rotation_rel, (10, 550), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                        '''
                        #Gemini is telling me that R_delta = R_aruco_in_world @ R_initial_in_world
                        #is mathematically incorrect. That could be why I am having to reverse signs manually.
                        #I will try transpose(T_initial_in_world) @ R_aruco_in_world instead
                        #In this case, transpose is same as inverse.
                        
                        #Pose of the aruco in world frame relative to where it started
                        delta_pose_world = T_current_aruco_in_world_frame[:3,3] - self.initial_T[:3,3]
                        R_initial_in_world = self.initial_T[:3,:3]
                        R_delta = R_aruco_in_world @ R_initial_in_world
                        r_delta = Rotation.from_matrix(R_delta)
                        yaw_world = np.degrees(np.arctan2(R_aruco_in_world[1,0], R_aruco_in_world[0,0]))
                        yaw_init = np.degrees(np.arctan2(R_initial_in_world[1,0], R_initial_in_world[0,0]))
                        yaw_change = yaw_world - yaw_init
                        roll_aruco_rel, pitch_aruco_rel, yaw_aruco_rel = r_delta.as_euler('xyz', degrees=True)
                        #text_aruco_translation_rel = f"Aruco relative to start X: {delta_pose_world[0]:.3f}, Y: {delta_pose_world[1]:.3f}, Z: {delta_pose_world[2]:.3f}"
                        text_aruco_translation_rel = f"Aruco relative to start X: {-delta_pose_world[1]:.3f}, Y: {delta_pose_world[0]:.3f}, Z: {delta_pose_world[2]:.3f}"
                        #text_aruco_rotation_rel = f"Aruco irelative to start Roll: {roll_aruco_rel:.3f} Pitch: {pitch_aruco_rel:.3f}, Yaw: {yaw_change:.3f}"
                        text_aruco_rotation_rel = f"Aruco relative to start Roll: {pitch_aruco_rel:.3f} Pitch: {-roll_aruco_rel:.3f}, Yaw: {-yaw_change:.3f}"
                        cv2.putText(frame_with_aruco, text_aruco_translation_rel, (10, 500), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                        cv2.putText(frame_with_aruco, text_aruco_rotation_rel, (10, 550), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                        
                        '''
                        #Pose of the aruco in world frame relative to where it started
                        delta_pose_world = T_current_aruco_in_world_frame[:3,3] - self.initial_T[:3,3]
                        R_initial_in_world = self.initial_T[:3,:3]
                        R_delta = np.linalg.inv(R_aruco_in_world) @ R_initial_in_world
                        r_delta = Rotation.from_matrix(R_delta)
                        yaw_world = np.degrees(np.arctan2(R_aruco_in_world[1,0], R_aruco_in_world[0,0]))
                        yaw_init = np.degrees(np.arctan2(R_initial_in_world[1,0], R_initial_in_world[0,0]))
                        yaw_change = yaw_world - yaw_init
                        roll_aruco_rel, pitch_aruco_rel, yaw_aruco_rel = r_delta.as_euler('xyz', degrees=True)
                        #text_aruco_translation_rel = f"Aruco relative to start X: {delta_pose_world[0]:.3f}, Y: {delta_pose_world[1]:.3f}, Z: {delta_pose_world[2]:.3f}"
                        text_aruco_translation_rel = f"Aruco relative to start X: {-delta_pose_world[0]:.3f}, Y: {-delta_pose_world[1]:.3f}, Z: {delta_pose_world[2]:.3f}"
                        #text_aruco_rotation_rel = f"Aruco irelative to start Roll: {roll_aruco_rel:.3f} Pitch: {pitch_aruco_rel:.3f}, Yaw: {yaw_change:.3f}"
                        text_aruco_rotation_rel = f"Aruco relative to start Roll: {-roll_aruco_rel:.3f} Pitch: {-pitch_aruco_rel:.3f}, Yaw: {-yaw_change:.3f}"
                        cv2.putText(frame_with_aruco, text_aruco_translation_rel, (10, 500), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                        cv2.putText(frame_with_aruco, text_aruco_rotation_rel, (10, 550), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                        '''

                        #Publish
                        #pose_msg = self.to_pose_stamped(T_relative)
                        #self.pub.publish(pose_msg)
                        pose_msg = self.make_pose_stamped(-delta_pose_world[1], delta_pose_world[0], delta_pose_world[2], pitch_aruco_rel, -roll_aruco_rel, -yaw_change)
                        self.pub.publish(pose_msg)
                #preview_frame_with_aruco = cv2.resize(frame_with_aruco, (960, 540))
                cv2.imshow("Aruco detection", frame_with_aruco)

            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                if len(self.pose_buffer) == self.pose_buffer.maxlen:
                    self.set_initial_pose()

        
        
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
    rospy.init_node("pose_tracking_node")
    #Read in calibration parameters for the cameras
    CAMERA_CALIB_PARAM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_params_home_webcam/4', 'calibration_data.pkl'))
    cameraMat1, distCoeffs1 = read_camera_params(CAMERA_CALIB_PARAM_DIR)
    FRAME_PARAM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_params_home_webcam/RobotBaseToCam/TransformationMatricesForStewCalib.pkl'))
    T_camera_to_world, T_world_to_camera = read_robotframe_calibration(FRAME_PARAM_DIR)
    tracker = Tracker(cameraMat1, distCoeffs1, T_camera_to_world, T_world_to_camera, camera_index=4, aruco_size=40)
    tracker.run()
    