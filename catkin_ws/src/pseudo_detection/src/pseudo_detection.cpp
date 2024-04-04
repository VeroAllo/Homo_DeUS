/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Homo DeUS. */

/**
 * @file
 *
 * @brief example on how to subscribe to an image topic and how to make the robot look towards a given direction
 *
 * How to test this application:
 *
 * 1) Launch the application:
 *
 *   $ rosrun zhomodeus pseudo_detection
 *
 * 2) Click on image pixels to make TIAGo look towards that direction
 *
 */

// C++ standard headers
#include <cmath>
#include <cstdlib>
#include <exception>
#include <string>

// Boost headers
// #include <boost/shared_ptr.hpp>

// ROS headers
#include <geometry_msgs/PoseStamped.h>
#include <homodeus_msgs/ObjectDetection.h>
#include <image_transport/image_transport.h>
#include <image_geometry/stereo_camera_model.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// OpenCV headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string WINDOW_NAME      = "Inside of TIAGo's head";
static const std::string CAMERA_FRAME     = "/xtion_rgb_optical_frame";
static const std::string IMAGE_TOPIC      = "/xtion/rgb/image_raw";
static const std::string DEPTH_TOPIC      = "/xtion/depth_registered/image_raw";
static const std::string CAMERA_INFO_TOPIC= "/xtion/rgb/camera_info";
static const std::string DETECTION_TOPIC  = "/Homodeus/Perception/Detect";

// Intrinsic parameters of the camera
cv::Mat cameraIntrinsics;

// https://docs.ros.org/en/api/image_geometry/html/c++/
image_geometry::PinholeCameraModel model_;

cv::Mat rgbImage;

cv::Mat depthImage;

ros::Time latestImageStamp;

ros::Publisher detection_pub;

int drawed_rectanle = 0, detected_object = 0;

cv::Point start_pnt, end_pnt;

int choice_object_detected = 0;         // 0: person, 1: object

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ROS call back for every new image received
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  latestImageStamp = imgMsg->header.stamp;

  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
  rgbImage = cvImgPtr->image.clone();
  if ( !drawed_rectanle )
    cv::imshow(WINDOW_NAME, cvImgPtr->image);
  
  auto key_pressed = cv::waitKey(50);                     // msecs
  if ( key_pressed == 112 && drawed_rectanle == 0 )       // 'p'
  {
    ROS_INFO("You choice to detecte a person.");
    choice_object_detected = 0;
  }
  else if ( key_pressed == 111 && drawed_rectanle == 0 )  // 'o'
  {
    ROS_INFO("You choice to detecte an object.");
    choice_object_detected = 1;
  }
}

// ROS call back for every new depth received
void depthCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  latestImageStamp = imgMsg->header.stamp;

  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::TYPE_32FC1);
  depthImage = cvImgPtr->image.clone();
}

void sendObjectDetection(ros::Time timestamp, std::string what_is, 
                         cv::Point start_pnt, cv::Point end_pnt)
{
  // 
  cv::Point up_left_pnt           = cv::Point(start_pnt.x < end_pnt.x ? start_pnt.x : end_pnt.x, 
                                              start_pnt.y < end_pnt.y ? start_pnt.y : end_pnt.y);
  cv::Point down_right_pnt        = cv::Point(start_pnt.x > end_pnt.x ? start_pnt.x : end_pnt.x, 
                                              start_pnt.y > end_pnt.y ? start_pnt.y : end_pnt.y);
  int width                       = std::abs(start_pnt.x - end_pnt.x);
  int height                      = std::abs(start_pnt.y - end_pnt.y);
  int padding_width               = width * 0.025;
  int padding_height              = height * 0.025;

  // Trouver la zone de l'image de profondeur associée a la zone de l'image rgb
  cv::Rect traget_box( cv::Point( up_left_pnt.x + padding_width, up_left_pnt.y + padding_height ), 
                       cv::Point( down_right_pnt.x - padding_width, down_right_pnt.y - padding_height ) );
  cv::Mat depth_box = depthImage( traget_box );
  cv::patchNaNs(depth_box, -1.);
  cv::Mat mask_not_nan = depth_box != -1.;
  float disparity = cv::mean( depthImage( traget_box ), mask_not_nan )[0];

  // Trouver les coordonnées 3D a partir des coordonnées 2D pixels
  cv::Point center_pnt = cv::Point( up_left_pnt.x + width / 2, 
                                    up_left_pnt.y + height / 2 );
  // Pin Hole Camera
  cv::Point3d point_3d = model_.projectPixelTo3dRay( center_pnt ) * disparity;
  ROS_INFO_STREAM("point_3d " << point_3d );
  float angle = std::atan2( point_3d.x, point_3d.z );

  // Ce que la perception pourrait traiter de plus
  //  soit la pose de l'objet selon le robot
  //  ou la carte (prendre en consideration une rotation de la tete p/r au tronc)
  tf2::Quaternion q;
  q.setRPY( 0, 0, -angle );
  q.normalize();

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  geometry_msgs::TransformStamped head_1_link_to_map;
  head_1_link_to_map = tf_buffer.lookupTransform("map", "head_1_link", ros::Time(0), ros::Duration(1.0) );

  geometry_msgs::PoseStamped poseRelative;
  poseRelative.header.seq         = 0;
  poseRelative.header.stamp       = latestImageStamp;
  poseRelative.header.frame_id    = "/head_1_link";

  poseRelative.pose.position.x    =  point_3d.z;   // distance objet-camera
  poseRelative.pose.position.y    = -point_3d.x;   // offset x
  poseRelative.pose.position.z    = -point_3d.y;   // offset y
  poseRelative.pose.orientation.x = q.getX();
  poseRelative.pose.orientation.y = q.getY();
  poseRelative.pose.orientation.z = q.getZ();
  poseRelative.pose.orientation.w = q.getW();

  // Initialize homodeus_msgs::ObjectDetection to send on topic
  homodeus_msgs::ObjectDetection objectDetected;
  // Prepare msg header
  objectDetected.header.seq       = 0;
  objectDetected.header.stamp     = timestamp;
  objectDetected.header.frame_id  = what_is;

  // Prepare msg box
  objectDetected.box.x            = up_left_pnt.x;
  objectDetected.box.y            = up_left_pnt.y;
  objectDetected.box.width        = width;
  objectDetected.box.height       = height;

  // Données géospatiales de l'objet détecté selon pose du robot
  objectDetected.distance         = disparity;
  objectDetected.angle            = angle;

  tf2::doTransform(poseRelative.pose, objectDetected.pose, head_1_link_to_map);

  detection_pub.publish( objectDetected );
}

// OpenCV callback function for mouse events on a window
void onMouse( int event, int u, int v, int, void* )
{
  if ( event != cv::EVENT_LBUTTONDOWN && 
       event != cv::EVENT_MOUSEMOVE && 
       event != cv::EVENT_LBUTTONUP )
      return;

  // Draw a rectangle on rgb image
  if ( event == cv::EVENT_LBUTTONDOWN && !drawed_rectanle )
  {
    // ROS_INFO("Start drawing a rectangle.");
    drawed_rectanle = 1;
    start_pnt = cv::Point(u, v);
  }
  else if ( drawed_rectanle )
  {
    // ROS_INFO("Continue drawing a rectangle.");
    end_pnt = cv::Point(u, v);
    if ( choice_object_detected == 0 )
      cv::rectangle(rgbImage, start_pnt, end_pnt, CV_RGB(0, 255, 0), 2, 8, 0);
    else
      cv::rectangle(rgbImage, start_pnt, end_pnt, CV_RGB(0, 0, 255), 2, 8, 0);
    cv::imshow(WINDOW_NAME, rgbImage);

    if ( event == cv::EVENT_LBUTTONUP )
    {
      // ROS_INFO("End drawing a rectangle.");
      drawed_rectanle = 0;
      detected_object = ( cv::waitKey(50) != 27 ) ? 1 : 0;
    }
  }

  // Break if not object as detected (not drawing rectangle)
  if( detected_object == 0 )
    return;
  detected_object = 0;

  // Break if box too small
  if(start_pnt == end_pnt)
    return;

  std::string what_is = ( choice_object_detected == 0 ) ? "person" : "object";
  sendObjectDetection( latestImageStamp, what_is, start_pnt, end_pnt );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "pseudo_detection");

  ROS_INFO("Starting pseudo_detection application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Get the camera intrinsic parameters from the appropriate ROS topic
  ROS_INFO("Waiting for camera intrinsics ... ");
  sensor_msgs::CameraInfoConstPtr cam_info = ros::topic::waitForMessage
      <sensor_msgs::CameraInfo>(CAMERA_INFO_TOPIC, ros::Duration(10.0));
  if(cam_info.use_count() > 0)
  {
    cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);
    cameraIntrinsics.at<double>(0, 0) = cam_info->K[0]; //fx
    cameraIntrinsics.at<double>(1, 1) = cam_info->K[4]; //fy
    cameraIntrinsics.at<double>(0, 2) = cam_info->K[2]; //cx
    cameraIntrinsics.at<double>(1, 2) = cam_info->K[5]; //cy
    cameraIntrinsics.at<double>(2, 2) = 1;
  }

  model_.fromCameraInfo(cam_info);
  ROS_INFO_STREAM("Model Camera is initialize ? " << model_.initialized() );

  // ROS_INFO_STREAM("Shape: " << model_.fullResolution());
  // ROS_INFO_STREAM("P: " << model_.projectionMatrix ());
  // ROS_INFO_STREAM("binning_x: " << model_.binningX());
  // ROS_INFO_STREAM("binning_y: " << model_.binningY());

  // Create the window to show TIAGo's camera images
  cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);

  // Set mouse handler for the window
  cv::setMouseCallback(WINDOW_NAME, onMouse);

  // Define ROS topic from where TIAGo publishes images
  image_transport::ImageTransport it(nh);
  // use compressed image transport to use less network bandwidth
  image_transport::TransportHints transportHint("compressed");

  ROS_INFO_STREAM("Subscribing to " << IMAGE_TOPIC << " ...");
  image_transport::Subscriber image_sub = it.subscribe(IMAGE_TOPIC, 1,
                                                       imageCallback, transportHint);

  ROS_INFO_STREAM("Subscribing to " << DEPTH_TOPIC << " ...");
  image_transport::Subscriber depth_sub = it.subscribe(DEPTH_TOPIC, 1,
                                                       depthCallback);

  // Define ROS topic to pusblih perceptron
  ROS_INFO_STREAM("Publishing to " << DETECTION_TOPIC << " ...");
  detection_pub = nh.advertise<homodeus_msgs::ObjectDetection>(DETECTION_TOPIC, 1);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  cv::destroyWindow(WINDOW_NAME);

  return EXIT_SUCCESS;
}
