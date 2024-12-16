#!/usr/bin/env python

# TODO PerceptionNode (FaceDetection)
import rospy
from geometry_msgs.msg import Pose, Point
from homodeus_msgs.msg import ObjectDetection, ObjectsDetection

def prepareMsgObjectDetection(what_is, point):
    # Initialize homodeus_msgs::ObjectDetection to send on topic
    objectDetected: ObjectDetection = ObjectDetection()

    # Prepare msg header
    objectDetected.header.seq       = 0
    objectDetected.header.stamp     = rospy.Time(0)
    objectDetected.header.frame_id  = what_is

    # Prepare msg box
    objectDetected.box.x            = 0
    objectDetected.box.y            = 0
    objectDetected.box.width        = 64   
    objectDetected.box.height       = 128

    # Données géospatiales de l'objet détecté selon pose du robot
    objectDetected.distance         = 0
    objectDetected.angle            = 0
    objectDetected.pose             = Pose()
    objectDetected.pose.position.x     = point.x
    objectDetected.pose.position.y     = point.y
    objectDetected.pose.position.z     = point.z

    return objectDetected

def PseudoProductDetection() -> None:
  rospy.init_node('ProductDetection', anonymous=True)
  pub = rospy.Publisher('/Homodeus/Perception/Detect/Product', ObjectsDetection, queue_size=10)
  rate = rospy.Rate(10) # 10hz

 
  labels = ['pomme', 'orange', 'fruits']
  # points = [Point(9.72, 6.69, 0.95), Point(9.54, 6.69, 0.96), Point(9.39, 6.72, 0.912)]
  points = [Point(9.13, 6.69, 0.95), Point(9.67, 6.68, 0.96), Point(10.2, 6.68, 0.912)]
  objectsDetected: ObjectsDetection = ObjectsDetection()
  objectsDetected.header.seq       = 0
  objectsDetected.header.stamp     = rospy.Time(0)
  objectsDetected.header.frame_id  = 'objects list'
  
  for idx, label in enumerate(labels):
      objectDetection: ObjectDetection = ObjectDetection()
      objectDetection = prepareMsgObjectDetection(label, points[idx])
      objectsDetected.objects.append(objectDetection)

  while not rospy.is_shutdown():
    pub.publish(objectsDetected)
    rate.sleep()

if __name__ == '__main__':
  try:
    PseudoProductDetection()
  except rospy.ROSInterruptException:
    pass