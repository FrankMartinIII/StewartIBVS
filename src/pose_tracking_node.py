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
    def __init__(self, camera_matrix, distortion_coefficients, camera_index=0, aruco_size=50):
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coefficients
        self.aruco_size = aruco_size
        self.pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=100)
         # Open camera
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            rospy.logerr("Cannot open camera")
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rospy.loginfo(f"Camera0 resolution: {self.width}x{self.height}")

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

    
    def run(self):
        rate = rospy.Rate(30)  # adjust publish rate

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to grab frame")
                continue
            preview_frame = cv2.resize(frame, (960, 540))
            cv2.imshow("Frame", preview_frame)
            grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(grey, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None:
                frame_with_aruco = frame.copy()
                #estimatePose returns tvec that describes transformation from aruco center to camera optical center, it does not return coordinates of aruco in cam frame
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.camera_matrix, self.distortion_coeffs)
                cv2.aruco.drawDetectedMarkers(frame_with_aruco, corners)
                for rvec, tvec in zip(rvecs, tvecs):
                    T_current = self.create_transformation_matrix(tvec, rvec)
                    if self.initial_T is None:
                        #Buffer of poses for averaging initial pose
                        self.pose_buffer.append(T_current)
                    else:
                        T_relative = np.linalg.inv(self.initial_T) @ T_current
                        R_current = T_current[:3,:3]
                        r_current = Rotation.from_matrix(R_current)
                        R_relative = T_relative[:3,:3]
                        r_relative = Rotation.from_matrix(R_relative)
                        roll_cam,pitch_cam,yaw_cam = r_current.as_euler('xyz', degrees=True)
                        roll_rel,pitch_rel,yaw_rel = r_relative.as_euler('xyz', degrees=True)
                        text_cam = f"CameraFrame X: {T_current[0,3]:.3f} mm  Y: {T_current[1,3]:.3f} mm  Z: {T_current[2,3]:.3f} mm"
                        text_cam_rot = f"Roll: {roll_cam:.3f} Pitch: {pitch_cam:.3f}, Yaw: {yaw_cam:.3f}"
                        #text_world = f"World FrameX: {marker_world_pos[0]:.3f} mm  Y: {marker_world_pos[1]:.3f} mm  Z: {marker_world_pos[2]:.3f} mm"
                        text_world = f"Relative to original X: {T_relative[0,3]:.3f} mm  Y: {T_relative[1,3]:.3f} mm  Z: {T_relative[2,3]:.3f} mm"
                        text_world_rot = f"Roll: {roll_rel:.3f} Pitch: {pitch_rel:.3f}, Yaw: {yaw_rel:.3f}"
                        cv2.putText(frame_with_aruco, text_cam, (10, self.height - 200), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 2)
                        cv2.putText(frame_with_aruco, text_cam_rot, (10, self.height - 150), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 2)
                        cv2.putText(frame_with_aruco, text_world, (10, self.height - 75), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 150), 2)
                        cv2.putText(frame_with_aruco, text_world_rot, (10, self.height - 30), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 150), 2)

                        #Publish
                        pose_msg = self.to_pose_stamped(T_relative)
                        self.pub.publish(pose_msg)
                preview_frame_with_aruco = cv2.resize(frame_with_aruco, (960, 540))
                cv2.imshow("Aruco detection", preview_frame_with_aruco)

            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                if len(self.pose_buffer) == self.pose_buffer.maxlen:
                    self.set_initial_pose()

        
        
def read_params(filePath):
    with open(filePath, 'rb') as f:
        data = pickle.load(f)
    cameraMatrix1 = data['cameraMatrix']
    distCoeffs1 = data['distortionCoeff']
    rotation_vecs = data['rotationVectors']
    translation_vecs = data['translationVectors']
    reprojection_err = data['reprojectionError']
    return cameraMatrix1, distCoeffs1

if __name__ == "__main__":
    rospy.init_node("pose_tracking_node")
    #Read in calibration parameters for the cameras
    PARAM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'calibration_params_home_webcam/0', 'calibration_data.pkl'))
    cameraMat1, distCoeffs1 = read_params(PARAM_DIR)
    tracker = Tracker(cameraMat1, distCoeffs1, camera_index=4, aruco_size=20)
    tracker.run()
    