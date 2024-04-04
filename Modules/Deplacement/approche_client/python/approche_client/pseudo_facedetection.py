#!/usr/bin/env python

# TODO PerceptionNode (FaceDetection)
import rospy
from homodeus_msgs.msg import BoundingBoxes, BoundingBox

def PseudoFaceDetection() -> None:
  pub = rospy.Publisher('proc_output_face_positions', _BoundingBoxes, queue_size=10)
  rospy.init_node('FaceDetection', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    frame_input = input('(x y width height): ')
    frame_face = list(map(int, frame_input.split(' ')))
    faces : _BoundingBoxes = _BoundingBoxes()
    faces.header.frame_id = 'xtion/rgb/image_raw'
    faces.header.stamp = rospy.Time(0)
    faces.boxes.append(BoundingBox(frame_face[0],frame_face[1],frame_face[2],frame_face[3]))
    pub.publish(faces)

if __name__ == '__main__':
  try:
    PseudoFaceDetection()
  except rospy.ROSInterruptException:
    pass