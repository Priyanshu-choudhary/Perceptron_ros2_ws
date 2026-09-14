import cv2
import numpy as np
import cv2.aruco as aruco
from collections import defaultdict
import Jetson.GPIO as GPIO
import time


class ArucoDetector:
    def __init__(self, calibration_file, marker_size=0.10, dictionary_type=aruco.DICT_7X7_1000):

        self.camera_matrix, self.dist_coeffs = self._load_camera_calibration(calibration_file)
        self.marker_size = marker_size
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary_type)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # Track marker detections across frames
        self.marker_history = defaultdict(list)
        self.required_consecutive_frames = 1

       
        
    def _load_camera_calibration(self, calibration_file):
        """Load camera calibration from YAML file"""
        fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise IOError(f"Cannot open calibration file: {calibration_file}")

        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
        fs.release()
        return camera_matrix, dist_coeffs

    def _estimate_marker_pose(self, marker_corners):
        """Estimate pose of a single marker"""
        obj_points = np.array([
            [-self.marker_size/2, self.marker_size/2, 0],
            [self.marker_size/2, self.marker_size/2, 0],
            [self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)

        ret, rvec, tvec = cv2.solvePnP(obj_points, marker_corners, 
                                      self.camera_matrix, self.dist_coeffs)
        return rvec, tvec

    def process_frame(self, frame):
        """
        Process a frame to detect ArUco markers.
        
        Args:
            frame: Input image frame
            
        Returns:
            dict: Dictionary of confirmed markers with their IDs as keys and 
                  (distance, position) as values. Only returns markers detected
                  in at least 3 consecutive frames.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, 
                                             parameters=self.aruco_params)
        
        confirmed_markers = {}
        
        if ids is not None:
            current_frame_markers = set()
            
            for i, marker_id in enumerate(ids.flatten()):
                # Estimate pose
                rvec, tvec = self._estimate_marker_pose(corners[i])
                distance = np.linalg.norm(tvec)
                position = tvec.flatten()
                
                # Add to current frame set
                current_frame_markers.add(marker_id)
                
                # Update marker history
                self.marker_history[marker_id].append((distance, position))
                
                # Keep only recent detections (up to required frames)
                if len(self.marker_history[marker_id]) > self.required_consecutive_frames:
                    self.marker_history[marker_id].pop(0)
                
                # Check if we have enough consecutive detections
                if len(self.marker_history[marker_id]) >= self.required_consecutive_frames:
                    # Get average of last n detections
                    distances = [d for d, _ in self.marker_history[marker_id]]
                    positions = [p for _, p in self.marker_history[marker_id]]
                    
                    avg_distance = np.mean(distances)
                    avg_position = np.mean(positions, axis=0)
                    
                    confirmed_markers[marker_id] = {
                        'distance': avg_distance,
                        'position': avg_position,
                        'rvec': rvec,
                        'tvec': tvec
                    }
            
            # Remove markers not detected in this frame
            for marker_id in list(self.marker_history.keys()):
                if marker_id not in current_frame_markers:
                    del self.marker_history[marker_id]
        
        if confirmed_markers:
            return confirmed_markers
        else:
            return None



class ArucoCamera:
    def __init__(self, calibration_file, sensor_id=0, flip_method=0):
        """
        Initialize the ArUco camera with GStreamer pipeline and LED indicator.
        """
        self.detector = ArucoDetector(calibration_file)
        # self.cap = cv2.VideoCapture(
        #     self._gstreamer_pipeline(sensor_id=sensor_id, flip_method=flip_method), 
        #     cv2.CAP_GSTREAMER
        # )
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            raise RuntimeError("Error: Could not open video capture")

        # LED setup (Pin 11 = GPIO 50)
        self.LED_PIN = 11
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.LED_PIN, GPIO.OUT, initial=GPIO.LOW)

        # Detection timing
        self.last_detection_time = time.time()
        self.detection_timeout = 0.6  # seconds

    def _gstreamer_pipeline(self, sensor_id=0, flip_method=0,
                          capture_width=1280, capture_height=720,
                          display_width=1280, display_height=720,
                          framerate=10):
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
            f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
            f"nvvidconv flip-method={flip_method} ! "
            f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
            f"videoconvert ! video/x-raw, format=(string)BGR ! appsink"
        )

    def get_markers(self):
        """
        Capture a frame and detect ArUco markers.
        Controls LED on Pin 11 to indicate detection status.
        """
        ret, frame = self.cap.read()
        if not ret:
            return None

        markers = self.detector.process_frame(frame)
        current_time = time.time()

        if markers:
            self.last_detection_time = current_time
            GPIO.output(self.LED_PIN, GPIO.HIGH)  # ON when detected
        else:
            if current_time - self.last_detection_time > self.detection_timeout:
                GPIO.output(self.LED_PIN, GPIO.LOW)  # OFF if not seen recently

        return markers

    def release(self):
        """Release camera and GPIO resources."""
        self.cap.release()
        GPIO.cleanup()


# Example usage
if __name__ == "__main__":
    # Initialize the detector
    calibration_file = "/mnt/usbdrive/jetson-home/camera/calibration_data/intrinsics.yml"
    aruco_cam = ArucoCamera(calibration_file)
    
    try:
        while True:
            # Get detected markers (only those seen in 3+ consecutive frames)
            markers = aruco_cam.get_markers()
            
            if markers:
                for marker_id, data in markers.items():
                    print(f"Marker {marker_id}:")
                    print(f"  Distance: {data['distance']:.2f}m")
                    print(f"  Position: {data['position']}")
                    print(f"  Rotation: {data['rvec'].flatten()}")
                    
                    # Here you would add your custom logic for when markers are detected
                    # For example:
                    # if marker_id == 42:
                    #     drone.move_toward(data['position'])
                    
            # Add your own break condition or leave running
            
    finally:
        aruco_cam.release()