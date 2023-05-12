#!/usr/bin/env python3
from ada_feeding_msgs.msg import FaceDetection
from ada_feeding_msgs.srv import ToggleFaceDetection
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from threading import Lock


class FaceDetectionNode(Node):
    def __init__(
        self,
        face_detection_interval=150,
        num_images_with_face=60,
        open_mouth_interval=90,
        num_images_with_open_mouth=30,
    ):
        """
        Initializes the FaceDetection node, which exposes a ToggleFaceDetection
        service that can be used to toggle the face detection on or off and
        publishes information about detected faces to the /face_detection
        topic when face detection is on.

        After face_detection_interval images without a face, this dummy function
        detects a face for num_images_with_face frames. After open_mouth_interval
        images with a face but without an open mouth, this dummy function
        detects an open mouth for num_images_with_open_mouth frames.

        Parameters:
        ----------
        face_detection_interval: The number of frames between each face detection.
        num_images_with_face: The number of frames that must have a face in them.
        open_mouth_interval: The number of frames between each open mouth detection.
        num_images_with_open_mouth: The number of frames that must have an open mouth in them.
        """
        super().__init__("face_detection")

        # Internal variables to track when a face and/or open mouth should be detected
        self.face_detection_interval = face_detection_interval
        self.num_images_with_face = num_images_with_face
        self.open_mouth_interval = open_mouth_interval
        self.num_images_with_open_mouth = num_images_with_open_mouth
        self.num_consecutive_images_without_face = 0
        self.num_consecutive_images_with_face = 0
        self.num_consecutive_images_without_open_mouth = 0
        self.num_consecutive_images_with_open_mouth = 0

        # Convert between ROS and CV images
        self.bridge = CvBridge()

        # Keeps track of whether face detection is on or not
        self.is_on = False
        self.is_on_lock = Lock()

        # Create the service
        self.srv = self.create_service(
            ToggleFaceDetection,
            "ToggleFaceDetection",
            self.toggle_face_detection_callback,
        )

        # Subscribe to the camera feed
        self.subscription = self.create_subscription(
            Image, "camera/color/image_raw", self.camera_callback, 1
        )
        self.subscription  # prevent unused variable warning

        # Create the publishers
        self.publisher_results = self.create_publisher(
            FaceDetection, "face_detection", 1
        )
        self.publisher_image = self.create_publisher(Image, "face_detection_img", 1)

    def toggle_face_detection_callback(self, request, response):
        """
        Callback function for the ToggleFaceDetection service. Safely toggles
        the face detection on or off depending on the request.
        """
        self.get_logger().info(
            "Incoming service request. turn_on: %s" % (request.turn_on)
        )
        if request.turn_on:
            # Reset counters
            self.num_consecutive_images_without_face = 0
            self.num_consecutive_images_with_face = 0
            self.num_consecutive_images_without_open_mouth = 0
            self.num_consecutive_images_with_open_mouth = 0
            # Turn on face detection
            self.is_on_lock.acquire()
            self.is_on = True
            self.is_on_lock.release()
            response.face_detection_is_on = True
        else:
            self.is_on_lock.acquire()
            self.is_on = False
            self.is_on_lock.release()
            response.face_detection_is_on = False
        return response

    def camera_callback(self, msg):
        """
        Callback function for the camera feed. If face detection is on, this
        function will detect faces in the image and publish information about
        them to the /face_detection topic.
        """
        self.is_on_lock.acquire()
        is_on = self.is_on
        self.is_on_lock.release()
        if is_on:
            # Update the number of consecutive images with/without a face
            is_face_detected = False
            if self.num_consecutive_images_with_face == self.num_images_with_face:
                self.num_consecutive_images_without_face = 0
                self.num_consecutive_images_with_face = 0
            if self.num_consecutive_images_without_face == self.face_detection_interval:
                # Detect a face
                self.num_consecutive_images_with_face += 1
                is_face_detected = True
            else:
                # Don't detect a face
                self.num_consecutive_images_without_face += 1

            # Update the number of consecutive images with/without an open mouth
            open_mouth_detected = False
            if is_face_detected:
                if (
                    self.num_consecutive_images_with_open_mouth
                    == self.num_images_with_open_mouth
                ):
                    self.num_consecutive_images_without_open_mouth = 0
                    self.num_consecutive_images_with_open_mouth = 0
                if (
                    self.num_consecutive_images_without_open_mouth
                    == self.open_mouth_interval
                ):
                    # Detect an open mouth
                    self.num_consecutive_images_with_open_mouth += 1
                    open_mouth_detected = True
                else:
                    # Don't detect an open mouth
                    self.num_consecutive_images_without_open_mouth += 1

            # Publish the face detection information
            face_detection_msg = FaceDetection()
            face_detection_msg.is_face_detected = is_face_detected
            if is_face_detected:
                # Add a dummy face marker to the sensor_msgs/Image
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.circle(
                    cv_image,
                    (msg.width // 2, msg.height // 2),
                    msg.height // 25,
                    (0, 0, 255),
                    -1,
                )
                annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                annotated_img = annotated_msg
                # Publish the detected mouth center
                face_detection_msg.detected_mouth_center = PointStamped()
                face_detection_msg.detected_mouth_center.header = msg.header
                face_detection_msg.detected_mouth_center.point.x = msg.width / 2.0
                face_detection_msg.detected_mouth_center.point.y = msg.height / 2.0
                face_detection_msg.detected_mouth_center.point.z = 0.0
            else:
                annotated_img = msg
            face_detection_msg.is_mouth_open = open_mouth_detected
            self.publisher_results.publish(face_detection_msg)
            self.publisher_image.publish(annotated_img)


def main(args=None):
    rclpy.init(args=args)

    face_detection = FaceDetectionNode()

    rclpy.spin(face_detection)

    rclpy.shutdown()


if __name__ == "__main__":
    main()