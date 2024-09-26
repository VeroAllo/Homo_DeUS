/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
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

/** \author Jordi Pages. */

// PAL headers
#include <homodeus_prehension/pcl_filters.hpp>
#include <homodeus_prehension/geometry.h>
#include <homodeus_prehension/tf_transforms.hpp>

// PCL headers
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
// Needed for clang linking
// https://github.com/PointCloudLibrary/pcl/issues/2406
#include <pcl/search/impl/search.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <homodeus_prehension/ObjectPoseSize.h>

// Eigen headers
#include <Eigen/Core>


//Mine 
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cmath>

#include <homodeus_msgs/BoundingBox.h>
#include <homodeus_msgs/ObjectDetection.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


namespace pal {

class ObjectDetector {

public:

  ObjectDetector(ros::NodeHandle& nh,
                   ros::NodeHandle& pnh);

  virtual ~ObjectDetector();

  void run();

protected:
  
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

  void publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
               const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
               const Eigen::Matrix4d& transform,
               double cylinderHeight,
               const std_msgs::Header& header);

  void publishPose(const geometry_msgs::Pose& pose,
                   const std_msgs::Header& header);

  void print(const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
             int numberOfPoints);

  /**
   * @brief getPointAndVector Given a point cloud in which a cylinder has been fit, it computes
   *        the main axis of the cylinder and provides the centroid of the point cloud projected on the
   *        main axis and the main axis director vector
   * @param cylinderCloud
   * @param linePoint
   * @param lineVector
   */
  void getPointAndVector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                         const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                         Eigen::Vector3d& linePoint,
                         Eigen::Vector3d& lineVector);

  /**
   * @brief computeHeight calculate height of cylinder provided its point cloud and the main axis vector
   *        and the cylinder centroid.
   * @param cylinderCloud
   * @param centroid
   * @param mainAxis
   * @return
   */
  double computeHeight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                       const Eigen::Vector3d& centroid,
                       const Eigen::Vector3d& mainAxis);

  void projectPointToLine(const Eigen::Vector3d& linePoint,
                          const Eigen::Vector3d& lineVector,
                          const Eigen::Vector3d& pointToProject,
                          Eigen::Vector3d& projectedPoint);

  void segmentationObject(const sensor_msgs::PointCloud2ConstPtr& cloud);

  void selectObject(geometry_msgs::Point pointBoundingBox, std::vector<geometry_msgs::Pose> pose_list);
  void objectDetectionCallback(const homodeus_msgs::ObjectDetection& objectDetectionMsg);
  double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
 
  void start();
  void stop();

  ros::NodeHandle& _nh, _pnh;
  ros::CallbackQueue _cbQueue;
  bool _enabled;
  double _rate;
  bool _dispos;

  // ROS interfaces
  ros::Subscriber _cloudSub;
  ros::Subscriber _objectDetectionSub;
  
  ros::Publisher  _cylinderCloudPub;
  ros::Publisher  _objectPosePub;
  ros::Publisher  _cylinderMarkerPub;
  ros::Publisher  _objectVisualisationMarkerPub;
  ros::Publisher  _objectPub;


  std::vector<geometry_msgs::Pose> _objets_pos_list;
};


ObjectDetector::ObjectDetector(ros::NodeHandle& nh,
                                   ros::NodeHandle& pnh):
  _nh(nh),
  _pnh(pnh),
  _enabled(false),
  _rate(5.0),
  _dispos(true)
{
  _nh.setCallbackQueue(&_cbQueue);

  pnh.param<double>("rate", _rate, _rate);

  _cylinderCloudPub  = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("cylinder_cloud", 1);
  _objectPosePub   = _pnh.advertise< geometry_msgs::PoseStamped >("object_pose", 1);
  _cylinderMarkerPub = _pnh.advertise<visualization_msgs::Marker>("marker", 1);
  _objectVisualisationMarkerPub = _pnh.advertise<visualization_msgs::MarkerArray>("object_marker", 1);
  _objectPub         = _pnh.advertise<object_recognition_msgs::RecognizedObjectArray>("recognized_objects",1);
}

ObjectDetector::~ObjectDetector()
{

}

void ObjectDetector::print(const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                             int numberOfPoints)
{
  std::stringstream ss;
  ss << std::endl << "Cylinder found with " << numberOfPoints << " points:  " << std::endl;
  ss << "\tRadius:      " << cylinderCoefficients->values[6] << " m" << std::endl;
  ss << "\tPoint:       (" <<
        cylinderCoefficients->values[0] << ", " <<
        cylinderCoefficients->values[1] << ", " <<
        cylinderCoefficients->values[2] << ")" << std::endl;
  ss << "\tAxis:        (" <<
        cylinderCoefficients->values[3] << ", " <<
        cylinderCoefficients->values[4] << ", " <<
        cylinderCoefficients->values[5] << ")" << std::endl;

  ROS_INFO_STREAM(ss.str());
}

void ObjectDetector::projectPointToLine(const Eigen::Vector3d& linePoint,
                                          const Eigen::Vector3d& lineVector,
                                          const Eigen::Vector3d& pointToProject,
                                          Eigen::Vector3d& projectedPoint)
{
  Eigen::Vector3d pointToLineVector;
  pointToLineVector = pointToProject - linePoint;
  projectedPoint = linePoint + (pointToLineVector.dot(lineVector) / lineVector.dot(lineVector) ) * lineVector;
}

void ObjectDetector::getPointAndVector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                                         const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                                         Eigen::Vector3d& linePoint,
                                         Eigen::Vector3d& lineVector)
{
  //compute cylinder centroid
  Eigen::Vector4d centroid;
  centroid.setZero();
  pcl::compute3DCentroid<pcl::PointXYZRGB>(*cylinderCloud, centroid);

  linePoint(0,0) = cylinderCoefficients->values[0];
  linePoint(1,0) = cylinderCoefficients->values[1];
  linePoint(2,0) = cylinderCoefficients->values[2];

  lineVector(0,0) = cylinderCoefficients->values[3];
  lineVector(1,0) = cylinderCoefficients->values[4];
  lineVector(2,0) = cylinderCoefficients->values[5];

  Eigen::Vector3d projectedCentroid;

  projectPointToLine(linePoint, lineVector,
                     centroid.head<3>(),
                     projectedCentroid);

  linePoint = projectedCentroid;
}

double ObjectDetector::computeHeight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                                       const Eigen::Vector3d& centroid,
                                       const Eigen::Vector3d& mainAxis)
{
  double height = 0;

  Eigen::Vector3d projectedPoint;
  //project all points into the main axis of the cylinder
  for (unsigned int i = 0; i < cylinderCloud->points.size(); ++i)
  {
    Eigen::Vector3d point(cylinderCloud->points[i].x,
                          cylinderCloud->points[i].y,
                          cylinderCloud->points[i].z);
    //take the largest distance to the centroid as the height of the cylinder
    projectPointToLine(centroid, mainAxis, point, projectedPoint);

    double distance = sqrt( (projectedPoint(0,0) - centroid(0,0))*(projectedPoint(0,0) - centroid(0,0)) +
                            (projectedPoint(1,0) - centroid(1,0))*(projectedPoint(1,0) - centroid(1,0)) +
                            (projectedPoint(2,0) - centroid(2,0))*(projectedPoint(2,0) - centroid(2,0)) );

    if ( 2*distance > height )
      height = 2*distance;
  }

  return height;
}

void ObjectDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  if ( (cloud->width * cloud->height) == 0)
    return;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pclCloud);

  // Old cilynder code
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCylinderCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients::Ptr cylinderCoefficients(new pcl::ModelCoefficients);
  bool found = pal::cylinderSegmentation<pcl::PointXYZRGB>(pclCloud,
                                                           pclCylinderCloud,
                                                           10,
                                                           0.015, 0.08,
                                                           cylinderCoefficients);
  
  //filter outliers in the cylinder cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclFilteredCylinderCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if ( pclCylinderCloud->empty() )
    pclFilteredCylinderCloud = pclCylinderCloud;
  else
    pal::statisticalOutlierRemoval<pcl::PointXYZRGB>(pclCylinderCloud, 25, 1.0,pclFilteredCylinderCloud);

  if ( found )
  {
    print(cylinderCoefficients, pclCylinderCloud->points.size());

    Eigen::Vector3d projectedCentroid, lineVector;
    getPointAndVector(pclFilteredCylinderCloud,
                      cylinderCoefficients,
                      projectedCentroid,
                      lineVector);

    double cylinderHeight = computeHeight(pclFilteredCylinderCloud,
                                          projectedCentroid,
                                          lineVector);

//    ROS_INFO_STREAM("The cylinder centroid is (" << centroid.head<3>().transpose() <<
//                    ") and projected to its axis is (" << projectedCentroid.transpose() << ")");

    Eigen::Matrix4d cylinderTransform;
    //create a frame given the cylinder parameters (point and vector)
    pal::pointAndLineTransform(lineVector,
                               projectedCentroid,
                               cylinderTransform);
    /*
    geometry_msgs::Pose pose;
    pal::convert(cylinderTransform, pose);
    std::cout <<  "Pose cylindre : " << pose << std::endl;

    publish(pclFilteredCylinderCloud,
        cylinderCoefficients,
        cylinderTransform,
        cylinderHeight,
        cloud->header);
        */

  }
  // =============================================================================================
  // New code for all shapes
  segmentationObject(cloud);
  
}

void ObjectDetector::segmentationObject(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud, *pclCloud);
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pclCloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pclCloud);
  ec.extract (cluster_indices);
  
  visualization_msgs::MarkerArray marker_array;
  int taille = cluster_indices.size();


  int j = 0;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*pclCloud)[idx]);
    } 

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_cluster);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    // Dimensions de la bounding box
    float largeur = max_point_OBB.x - min_point_OBB.x;
    float Longueur = max_point_OBB.y - min_point_OBB.y;
    float Hauteur = max_point_OBB.z - min_point_OBB.y;

    std::cout << "Largeur : " << max_point_OBB.x - min_point_OBB.x << std::endl;
    std::cout << "Longueur : " << max_point_OBB.y - min_point_OBB.y << std::endl;
    std::cout << "Hauteur : " << max_point_OBB.z - min_point_OBB.z << std::endl;

    //Eigen::Vector3f position_world = transform_matrix.block<3, 3>(0, 0) * position_OBB.getVector3fMap() + centroid.head<3>();
    std::cout << "Position dans le monde : " << position_OBB.x << ", " << position_OBB.y << ", " << position_OBB.z << std::endl;

    // TODO : VERIFIER quaternions
    Eigen::Quaternionf quaternionff;
    quaternionff = Eigen::Quaternionf(rotational_matrix_OBB);
    geometry_msgs::Point point;
    point.x = position_OBB.x;
    point.y = position_OBB.y;
    point.z = position_OBB.z;

    geometry_msgs::Quaternion quaternion;
    
    // Quaternion not working yet
    quaternion.x = 0;//quaternionff.x();
    quaternion.y = 0;//quaternionff.y();
    quaternion.z = 0;//quaternionff.z();
    quaternion.w = 1;//quaternionff.w();

    // std::cout << "rotational_matrix_OBB : " << rotational_matrix_OBB << std::endl;
    // std::cout << "quaternion : " << quaternion << std::endl;
    

    geometry_msgs::Pose pose;
    pose.position = point;
    pose.orientation = quaternion;

    _objets_pos_list.push_back(pose);

    std::string string_var;

    // Afficher la bounding box 3D 

    visualization_msgs::Marker marker;
    //marker.action = visualization_msgs::Marker::ADD;

    marker.header = cloud->header;
    marker.id = j;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose = pose;
    marker.scale.x = 0.005f;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();


    float max_x = max_point_OBB.z;
    float max_y = max_point_OBB.y;
    float max_z = max_point_OBB.x;

    float min_x = min_point_OBB.z;
    float min_y = min_point_OBB.y;
    float min_z = min_point_OBB.x;

    float width = max_x - min_x;
    float length = max_y - min_y;
    float height = max_z - min_z;

    geometry_msgs::Point p;
    // ===========================
    p.x = max_x;
    p.y = max_y;
    p.z = max_z;

    marker.points.push_back(p);

    p.x = max_x - width;
    p.y = max_y;
    p.z = max_z;

    marker.points.push_back(p);

    // ===========================
    p.x = max_x;
    p.y = max_y;
    p.z = max_z;

    marker.points.push_back(p);

    p.x = max_x;
    p.y = max_y - length;
    p.z = max_z;

    marker.points.push_back(p);

    // ===========================
    p.x = max_x - width;
    p.y = max_y;
    p.z = max_z;

    marker.points.push_back(p);

    p.x = max_x - width;
    p.y = max_y - length;
    p.z = max_z;

    marker.points.push_back(p);

    // ===========================
    p.x = max_x;
    p.y = max_y - length;
    p.z = max_z;

    marker.points.push_back(p);

    marker.points.push_back(p);

    // ===========================
    p.x = max_x;
    p.y = max_y - length;
    p.z = max_z;

    marker.points.push_back(p);

    p.x = max_x;
    p.y = max_y - length;
    p.z = max_z - height;

    marker.points.push_back(p);

    // +++++++++++++++++++++++++++++++

    // ===========================
    p.x = min_x;
    p.y = min_y;
    p.z = min_z;

    marker.points.push_back(p);

    p.x = min_x + width;
    p.y = min_y;
    p.z = min_z;

    marker.points.push_back(p);

    // ===========================
    p.x = min_x;
    p.y = min_y;
    p.z = min_z;

    marker.points.push_back(p);

    p.x = min_x;
    p.y = min_y + length;
    p.z = min_z;

    marker.points.push_back(p);

    // ===========================
    p.x = min_x + width;
    p.y = min_y;
    p.z = min_z;

    marker.points.push_back(p);

    p.x = min_x + width;
    p.y = min_y + length;
    p.z = min_z;

    marker.points.push_back(p);

    // ===========================
    p.x = min_x;
    p.y = min_y + length;
    p.z = min_z;

    marker.points.push_back(p);

    p.x = min_x + width;
    p.y = min_y + length;
    p.z = min_z;

    marker.points.push_back(p);

    // ===========================
    p.x = min_x;
    p.y = min_y;
    p.z = min_z;

    marker.points.push_back(p);

    p.x = min_x;
    p.y = min_y;
    p.z = min_z + height;

    marker.points.push_back(p);


    marker_array.markers.push_back(marker);
    j = j + 1;
  }

  // Publication de la position de l'objet a prendre
  // TODO : Changer la condition pour trouver lequel des objets il faut prendre

  // publishPose(_objets_pos_list[0], cloud->header);

  std::cout << "Nombre dobjets taille: " << taille << std::endl;
  
  if ( _objectVisualisationMarkerPub.getNumSubscribers() > 0 )
  {
    _objectVisualisationMarkerPub.publish(marker_array);
  }

}

void ObjectDetector::publishPose(const geometry_msgs::Pose& pose,
                                   const std_msgs::Header& header)
{
  if ( _objectPosePub.getNumSubscribers() > 0)
  {
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.pose   = pose;
    poseMsg.header = header;
    _objectPosePub.publish(poseMsg);
  }
}

void ObjectDetector::selectObject(geometry_msgs::Point pointBoundingBox, std::vector<geometry_msgs::Pose> pose_list){
  double small_dist = 999999999; 

  geometry_msgs::Pose pose_to_grasp;
  
  for (geometry_msgs::Pose pose : pose_list){
    double dist = calculateDistance(pose.position, pointBoundingBox);
    if (dist < small_dist) {
      small_dist = dist;
      pose_to_grasp = pose;
    }
  }

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "base_link";
  publishPose(pose_to_grasp, header);
}

void ObjectDetector::objectDetectionCallback(const homodeus_msgs::ObjectDetection& objectDetectionMsg) {

  geometry_msgs::Pose pose = objectDetectionMsg.pose;
  geometry_msgs::PointStamped point_in_map;
  point_in_map.header = objectDetectionMsg.header;
  point_in_map.point = pose.position;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("base_link", "base_footprint", ros::Time(0), ros::Duration(1.0));

  geometry_msgs::PointStamped point_in_base_link;
  tf2::doTransform(point_in_map, point_in_base_link, transformStamped);

  // Afficher le point dans le référentiel /base_link
  ROS_INFO("Point in /base_link: [%.2f, %.2f, %.2f]", point_in_base_link.point.x, point_in_base_link.point.y, point_in_base_link.point.z);

  selectObject(point_in_base_link.point, _objets_pos_list);
}

double ObjectDetector::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  pcl::PointXYZ pcl_point1 = pcl::PointXYZ(p1.x, p1.y, p1.z);
  pcl::PointXYZ pcl_point2 = pcl::PointXYZ(p2.x, p2.y, p2.z);

  return pcl::euclideanDistance(pcl_point1, pcl_point2);
}

// Deprecated
void ObjectDetector::publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                               const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                               const Eigen::Matrix4d& transform,
                               double cylinderHeight,
                               const std_msgs::Header& header)
{
  
  if ( _cylinderCloudPub.getNumSubscribers() > 0 )
  {
    pcl_conversions::toPCL(header, cylinderCloud->header);
    _cylinderCloudPub.publish(cylinderCloud);
  }
  
  geometry_msgs::Pose pose;
  pal::convert(transform, pose);
  /*
  if ( _objectPosePub.getNumSubscribers() > 0 )
    publishPose(pose, header);
  */
  
  if ( _cylinderMarkerPub.getNumSubscribers() > 0 )
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose = pose;
    marker.scale.x = cylinderCoefficients->values[6]*2;
    marker.scale.y = cylinderCoefficients->values[6]*2;
    marker.scale.z = cylinderHeight;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    _cylinderMarkerPub.publish(marker);
  }

  
  if ( _objectPub.getNumSubscribers() > 0 )
  {
    object_recognition_msgs::RecognizedObjectArray objects;
    objects.header = header;
    object_recognition_msgs::RecognizedObject object;
    object.pose.pose.pose = pose;
    object.pose.header = header;
    objects.objects.push_back(object);
    _objectPub.publish(objects);
  }
  
}


void ObjectDetector::start()
{
  _cloudSub = _nh.subscribe("cloud", 1, &ObjectDetector::cloudCallback, this);

  _objectDetectionSub = _nh.subscribe("/Homodeus/Perception/Detect", 1, &ObjectDetector::objectDetectionCallback, this);
  _enabled = true;
}

void ObjectDetector::stop()
{
  _cloudSub.shutdown();
  _objectDetectionSub.shutdown();
  _enabled = false;
}

void ObjectDetector::run()
{
  ros::Rate loopRate(_rate);

  double halfPeriod = 0.5*1.0/_rate;

  while ( ros::ok() )
  {
    bool anySubscriber = _objectPosePub.getNumSubscribers() > 0 ||
                         _cylinderCloudPub.getNumSubscribers() > 0 ||
                         _cylinderMarkerPub.getNumSubscribers() > 0 ||
                         _objectVisualisationMarkerPub.getNumSubscribers() > 0 ||
                         _objectPub.getNumSubscribers() > 0;


    if ( !_enabled && anySubscriber )
    {
      ROS_INFO("Enabling node because there are subscribers");
      start();
    }
    else if ( _enabled && !anySubscriber )
    {
      ROS_INFO("Disabling node because there are no subscribers");
      stop();
    }

    //check for subscriber's callbacks
    _cbQueue.callAvailable(ros::WallDuration(halfPeriod));

    loopRate.sleep();
  }
}


}

int main(int argc, char**argv)
{
  ros::init (argc, argv, "object_detector");

  ros::NodeHandle nh, pnh("~");

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  pal::ObjectDetector detector(nh, pnh);

  detector.run();

  return 0;
}
