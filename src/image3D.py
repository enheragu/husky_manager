#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PointCloudSubscriber:
    def __init__(self, fx,fy,dy,dx):
        rospy.init_node('cloud_to_image_node')
        self.in_topic = rospy.get_param("~pointcloud_topic", "/ouster/points")
        self.out_topic = rospy.get_param("~output_topic", "/ouster/image3d")
        
        self.subscriber = rospy.Subscriber(self.in_topic, PointCloud2, self.callback)
        self.image_pub = rospy.Publisher(self.out_topic, Image, queue_size=1)
        
        self.fx=fx
        self.fy=fy
        self.dy=dy
        self.dx=dx
        self.cx = self.dx/2
        self.cy = self.dy/2
        self.bridge = CvBridge()
        self.points_3d=None
        self.Tlc=None
        self.data_received=False
        self.camera_matrix = np.array([[self.fx, 0, self.cx],
                                  [0, self.fy, self.cy],
                                  [0, 0, 1]], np.float32)

        self.points_robot_lidar = np.array([
            [-0.75, -0.75, -0.5],
            [-0.75, -0.75, -1],
            [-0.75, 0.75, -0.5],
            [-0.75, 0.75, -1],
            [0.75, 0.75, -0.5],
            [0.75, 0.75, -1],
            [0.75, -0.75, -0.5],
            [0.75, -0.75, -1]
        ])


    def publish_image(self, imagen):
        try:
            # Convertir imagen OpenCV a mensaje ROS
            imagen_convertida = (imagen * 255).astype(np.uint8)
            imagen_msg = self.bridge.cv2_to_imgmsg(imagen_convertida, encoding="bgr8")
            self.image_pub.publish(imagen_msg)
        except Exception as e:
            rospy.logerr(e)

    def callback(self, data):
        # Convertir PointCloud2 a una lista de puntos
        pc_list = []
        for point in pc2.read_points(data, skip_nans=True):
            pc_list.append([point[0], point[1], point[2]])
        self.points_3d = np.asarray(pc_list)
        self.data_received=True



    def project2image(self,points):

        dist_coeffs = np.zeros((5, 1), np.float32)
        tvec = np.zeros((3, 1), np.float32)
        rvec = np.zeros((3, 1), np.float32)
        Tcl = np.linalg.inv(self.Tlc)
        self.points_camera=[]
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        points_transformed = np.dot(points_homogeneous, Tcl.T)
        self.points_camera = points_transformed

        points_camera2=self.points_camera[:,0:3]
        prueba=np.copy(points_camera2)
        points_2d, _ = cv2.projectPoints(prueba,rvec, tvec, self.camera_matrix,dist_coeffs)
        image_size = (self.dx, self.dy,3)
        image = np.zeros((image_size[1], image_size[0],image_size[2]))
        for point in points_2d.squeeze().astype(int):
            cv2.circle(image, tuple(point), 1, (1, 0.8784, 0.8784), -1)

        return image


    def draw_robot(self):
        dist_coeffs = np.zeros((5, 1), np.float32)
        tvec = np.zeros((3, 1), np.float32)
        rvec = np.zeros((3, 1), np.float32)
        Tcl = np.linalg.inv(self.Tlc)
        points2=[]
        points_homogeneous = np.hstack((self.points_robot_lidar, np.ones((self.points_robot_lidar.shape[0], 1))))
        points_transformed = np.dot(points_homogeneous, Tcl.T)
        points_robot_camera = points_transformed
        points_robot_camera2 = points_robot_camera[:, 0:3]
        prueba = np.copy(points_robot_camera2)
        points_2d, _ = cv2.projectPoints(prueba, rvec, tvec, self.camera_matrix, dist_coeffs)
        return points_2d
    def rotate_y(self, angle_degrees):
        angle_radians = np.radians(angle_degrees)
        cos_theta = np.cos(angle_radians)
        sin_theta = np.sin(angle_radians)

        R_y = np.array([
            [cos_theta, 0, sin_theta, 0],
            [0, 1, 0, 0],
            [-sin_theta, 0, cos_theta, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)

        # Apply the rotation to the existing transformation matrix
        self.Tlc_rotated = np.dot(R_y, self.Tlc)
        return self.Tlc_rotated


    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.data_received:
                idx=np.where((self.points_3d[:,0]>-3) & (self.points_3d[:,0]<15))
                angle_degrees = 35  # Rotate 45 degrees around the Y-axis
                self.Tlc = np.array([[0,0,1,-10],[-1., 0, 0, 0],[0,-1,0,0], [0,0,0,1]])
                self.Tlc = self.rotate_y(angle_degrees)
                image_pointcloud=self.project2image(self.points_3d[idx])
                # idx4=np.where(image_pointcloud[:,:,0]!=1)[0]
                # idx5 = np.where(image_pointcloud[:,:,0]!=1)[1]
                # image_pointcloud[idx4,idx5]=np.array([1,1,1])
                points_2d = self.draw_robot().squeeze().astype(int)
                row=points_2d[:,0]
                col=points_2d[:,1]

                image2 = cv2.line(image_pointcloud, (row[0],col[0]), (row[1],col[1]), (1,1,0), 2)
                image2 = cv2.line(image_pointcloud, (row[2], col[2]), (row[3], col[3]), (1, 1, 0), 2)
                image2 = cv2.line(image_pointcloud, (row[0], col[0]), (row[2], col[2]), (1, 1, 0), 2)
                image2 = cv2.line(image_pointcloud, (row[1], col[1]), (row[3], col[3]), (1, 1, 0), 2)
                image2 = cv2.line(image_pointcloud, (row[0],col[0]), (row[6],col[6]), (1,1,0), 2)
                image2 = cv2.line(image_pointcloud, (row[2],col[2]), (row[4],col[4]), (1,1,0), 2)
                image2 = cv2.line(image_pointcloud, (row[6],col[6]), (row[7],col[7]), (1,1,0), 2)
                image2 = cv2.line(image_pointcloud, (row[7],col[7]), (row[5],col[5]), (1,1,0), 2)
                image2 = cv2.line(image_pointcloud, (row[5],col[5]), (row[3],col[3]), (1,1,0), 2)
                image2 = cv2.line(image_pointcloud, (row[4],col[4]), (row[5],col[5]), (1,1,0), 2)
                image2 = cv2.line(image_pointcloud, (row[7],col[7]), (row[1],col[1]), (1,1,0), 2)
                image2 = cv2.line(image_pointcloud, (row[6],col[6]), (row[4],col[4]), (1,1,0), 2)

                self.publish_image(image_pointcloud)
                rate.sleep()

def main():
    try:
        # pc_subscriber = PointCloudSubscriber(fx=750, fy=750, dy=300, dx=1000)
        pc_subscriber = PointCloudSubscriber(fx=600, fy=600, dy=300, dx=1000)
        pc_subscriber.run()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
