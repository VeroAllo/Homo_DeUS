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

// PCL headers
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/distances.h>
// Needed for clang linking
// https://github.com/PointCloudLibrary/pcl/issues/2406
#include <pcl/search/impl/search.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

// ROS headers
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

// Eigen headers
#include <Eigen/Core>


//HomodeUS
#include <homodeus_msgs/BoundingBox.h>
#include <homodeus_msgs/ObjectDetection.h>
#include <homodeus_prehension/PrehensionPos.h>
#include <homodeus_msgs/HDBoundingBox.h>
#include <homodeus_msgs/HDStatus.h>


namespace pal {

class ObjectDetector {

public:

  ObjectDetector(ros::NodeHandle& nh,
                   ros::NodeHandle& pnh);

  virtual ~ObjectDetector();

  void run();

protected:
  
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void planeCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

  void publishPickPose(const geometry_msgs::Pose& pose,
                      const homodeus_msgs::DesireID& desireID,
                      std::vector<moveit_msgs::CollisionObject> obstacles_list);

  void publishDropPose(const homodeus_msgs::DesireID& desireID,
                      std::vector<moveit_msgs::CollisionObject> obstacles_list);
  void publishPickPosestamped(const geometry_msgs::Pose& pose);

  void selectObject(geometry_msgs::Point pointBoundingBox, std::vector<geometry_msgs::Pose> pose_list);
  void pickObjectCallback(const homodeus_msgs::ObjectDetection& objectDetectionMsg);
  void dropObjectCallback(const homodeus_msgs::HDBoundingBox& boundingBoxMsg);
  double distanceBetweenPositions(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
  geometry_msgs::Point vectorToPoint(const Eigen::Vector3f& point);
  visualization_msgs::Marker createBoundingBox(std_msgs::Header header, geometry_msgs::Pose pose, std_msgs::ColorRGBA color, pcl::PointXYZ max_point_OBB, pcl::PointXYZ min_point_OBB, int id, bool isPlan);
  moveit_msgs::CollisionObject createObstacles(float Longueur, float largeur, float Hauteur, pcl::PointXYZ position_OBB, geometry_msgs::Quaternion quaternion, std::string id);


  void start();
  void stop();

  ros::NodeHandle& _nh, _pnh;
  ros::CallbackQueue _cbQueue;
  double _rate;

  // ROS interfaces
  ros::Subscriber _cloudSub;
  ros::Subscriber _planCloudSub;
  ros::Subscriber _objectPickSub;
  ros::Subscriber _objectDropSub;
  
  ros::Publisher  _prehension_pick_pose;
  ros::Publisher  _prehension_drop_pose;
  ros::Publisher  _objectVisualisationPosePub;
  ros::Publisher  _objectVisualisationMarkerPub;
  ros::Publisher  _planeVisualisationMarkerPub;
  
  ros::Publisher _hbba_take_status_pub;

  std::vector<geometry_msgs::Pose> _objets_pos_list;
  std::vector<moveit_msgs::CollisionObject> _obstacle_list;
  std::vector<moveit_msgs::CollisionObject> _obstacle_plan_list;
};


ObjectDetector::ObjectDetector(ros::NodeHandle& nh,
                                   ros::NodeHandle& pnh):
  _nh(nh),
  _pnh(pnh),
  _rate(5.0)
{
  _nh.setCallbackQueue(&_cbQueue);

  pnh.param<double>("rate", _rate, _rate);

  _prehension_pick_pose   = _pnh.advertise<homodeus_prehension::PrehensionPos>("prehension_pose", 1);
  _prehension_drop_pose = _pnh.advertise<homodeus_prehension::PrehensionPos>("prehension_drop_pose", 1);
  _objectVisualisationPosePub   = _pnh.advertise< geometry_msgs::PoseStamped >("object_pose", 1);
  _objectVisualisationMarkerPub = _pnh.advertise<visualization_msgs::MarkerArray>("object_marker", 1);
  _planeVisualisationMarkerPub = _pnh.advertise<visualization_msgs::MarkerArray>("plane_marker", 1);

  _hbba_take_status_pub = nh.advertise<homodeus_msgs::HDStatus>("/Homodeus/Behaviour/Take/Status", 1);
  
}

ObjectDetector::~ObjectDetector()
{

}


void ObjectDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud, *pclCloud);
  
  if (pclCloud->empty()){
    ROS_INFO("OBJECTS : Cloud EMPTY ");
    return;
  }

  // Seperating cloud into clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pclCloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (5000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pclCloud);
  ec.extract (cluster_indices);
  
  visualization_msgs::MarkerArray marker_array;

  int j = 0;
  _objets_pos_list.clear();
  _obstacle_list.clear();

  // Create a object for each cluster
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*pclCloud)[idx]);
    } 

    // Extracting object dimensions and pose
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_cluster);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    
    float largeur = max_point_OBB.x - min_point_OBB.x;
    float Longueur = max_point_OBB.y - min_point_OBB.y;
    float Hauteur = max_point_OBB.z - min_point_OBB.z;

    Eigen::Quaternionf quaternionff;
    quaternionff = Eigen::Quaternionf(rotational_matrix_OBB);
    geometry_msgs::Point point;
    point.x = position_OBB.x;
    point.y = position_OBB.y;
    point.z = position_OBB.z;

    geometry_msgs::Quaternion quaternion;
    geometry_msgs::Pose pose;
    pose.position = point;

    pose.position = point;
    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.0f;
    color.g = 0.0f;
    color.b = 1.0f;

    // Set the quaternion to keep the object straight
    quaternion.x = 0.5;   // quaternion.x = quaternionff.x();
    quaternion.y = 0.5;   // quaternion.y = quaternionff.y();
    quaternion.z = 0.5;   // quaternion.z = quaternionff.z();
    quaternion.w = -0.5;  // quaternion.w = quaternionff.w();
    pose.orientation = quaternion;
    
    // Create markers to visualise objects on RVIZ
    marker = createBoundingBox(cloud->header, pose, color, max_point_OBB, min_point_OBB, j, false);
    marker_array.markers.push_back(marker);
    
    // Add new object to the lists
    _objets_pos_list.push_back(pose);
    std::string obstacle_id = "obstacle_" + std::to_string(j);
    _obstacle_list.push_back(createObstacles(Longueur, largeur, Hauteur, position_OBB, quaternion, obstacle_id));
    j = j + 1;
  }

  if ( _objectVisualisationMarkerPub.getNumSubscribers() > 0 )
  {
    _objectVisualisationMarkerPub.publish(marker_array);
  }
}

void ObjectDetector::planeCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud, *pclCloud);
  
  if (pclCloud->empty()){
    ROS_INFO("PLANE : Cloud EMPTY ");
    return;
  }

  // Seperating cloud into clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pclCloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (50000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pclCloud);
  ec.extract (cluster_indices);
  
  visualization_msgs::MarkerArray marker_array;

  int j = 0;
  _obstacle_plan_list.clear();


  // Create a plane for each cluster
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*pclCloud)[idx]);
    } 

    // Extracting plane dimensions and pose
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_cluster);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    float largeur = max_point_OBB.x - min_point_OBB.x;
    float Longueur = max_point_OBB.y - min_point_OBB.y;
    float Hauteur = max_point_OBB.z - min_point_OBB.z;

    // Adding a little more for safety
    largeur += 0.07;
    Longueur += 0.07;

    Eigen::Quaternionf quaternionff;
    quaternionff = Eigen::Quaternionf(rotational_matrix_OBB);

    geometry_msgs::Point point;
    point.x = position_OBB.x;
    point.y = position_OBB.y;
    point.z = position_OBB.z;

    geometry_msgs::Quaternion quaternion;

    geometry_msgs::Pose pose;
    pose.position = point;
    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 1.0f;
    color.g = 0.0f;
    color.b = 0.0f;

    // Set the quaternion to keep the plane straight
    quaternion.x = 0;     // quaternion.x = quaternionff.x();
    quaternion.y = 0;     // quaternion.y = quaternionff.y();
    quaternion.z = -0.7;  // quaternion.z = quaternionff.z();
    quaternion.w = 0.7;   // quaternion.w = quaternionff.w();
    pose.orientation = quaternion;

    // Create markers to visualise planes on RVIZ
    marker = createBoundingBox(cloud->header, pose, color, max_point_OBB, min_point_OBB, j, true);
    marker_array.markers.push_back(marker);

    // Add new plane to the list
    std::string obstacle_id = "plane_" + std::to_string(j++);
    _obstacle_plan_list.push_back(createObstacles(Longueur, largeur, Hauteur, position_OBB, quaternion, obstacle_id));

    j = j + 1;
  }

  if ( _planeVisualisationMarkerPub.getNumSubscribers() > 0 )
  {
    _planeVisualisationMarkerPub.publish(marker_array);
  }
}

void ObjectDetector::publishPickPose(const geometry_msgs::Pose& pose, const homodeus_msgs::DesireID& desireID, std::vector<moveit_msgs::CollisionObject> obstacles_list)
{
  if ( _prehension_pick_pose.getNumSubscribers() > 0)
  {
    homodeus_msgs::HDPose hd_pos_msg;
    hd_pos_msg.id = desireID;
    hd_pos_msg.pose = pose;

    homodeus_prehension::PrehensionPos prehension_pos_msg;
    prehension_pos_msg.hdpose = hd_pos_msg;
    prehension_pos_msg.obstacles = obstacles_list;

    _prehension_pick_pose.publish(prehension_pos_msg);
  }
}

void ObjectDetector::publishDropPose(const homodeus_msgs::DesireID& desireID, std::vector<moveit_msgs::CollisionObject> obstacles_list)
{
  if ( _prehension_drop_pose.getNumSubscribers() > 0)
  {
    homodeus_msgs::HDPose hd_pos_msg;
    hd_pos_msg.id = desireID;

    homodeus_prehension::PrehensionPos prehension_pos_msg;
    prehension_pos_msg.hdpose = hd_pos_msg;
    prehension_pos_msg.obstacles = obstacles_list;

    _prehension_drop_pose.publish(prehension_pos_msg);
  }
}

void ObjectDetector::publishPickPosestamped(const geometry_msgs::Pose& pose)
{
  std_msgs::Header headerA;
  headerA.stamp = ros::Time::now();
  headerA.frame_id = "base_link";
  headerA.seq = 1; 

  geometry_msgs::PoseStamped posestamped;
  posestamped.pose = pose;
  posestamped.header = headerA;
  _objectVisualisationPosePub.publish(posestamped); 
}

void ObjectDetector::pickObjectCallback(const homodeus_msgs::ObjectDetection& objectDetectionMsg) {

  ROS_INFO_STREAM("ObjectDetectionMSG RECEIVED");
  if (objectDetectionMsg.header.frame_id == "NOT FOUND") {
    ROS_INFO("NO OBJECT TO TAKE");
    homodeus_msgs::HDStatus hd_status_msg;
    hd_status_msg.id =  objectDetectionMsg.id;
    _hbba_take_status_pub.publish(hd_status_msg);
    return;
  }
  geometry_msgs::Pose pose = objectDetectionMsg.pose;
  geometry_msgs::PointStamped point_in_map;
  point_in_map.header = objectDetectionMsg.header;
  point_in_map.point = pose.position;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // Transforming the position into base_link frame.
  geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));

  geometry_msgs::PointStamped point_in_base_link;
  tf2::doTransform(point_in_map, point_in_base_link, transformStamped);

  // Select Object
  double small_dist = 999999999; 

  geometry_msgs::Pose pose_to_grasp;

  int idx = 0;
  int i = 0;
  for (geometry_msgs::Pose poseTemp : _objets_pos_list){
    double dist = distanceBetweenPositions(poseTemp.position, point_in_base_link.point);
    if (dist < small_dist) {
      small_dist = dist;
      pose_to_grasp = poseTemp;
      idx = i;
    }
    i++;
  }
  std::vector<moveit_msgs::CollisionObject> obstacles_copy = _obstacle_list;

  obstacles_copy.erase(obstacles_copy.begin() + idx);
  obstacles_copy.insert(obstacles_copy.end(), _obstacle_plan_list.begin(), _obstacle_plan_list.end());

  publishPickPose(pose_to_grasp, objectDetectionMsg.id, obstacles_copy);  
  publishPickPosestamped(pose_to_grasp);
}
void ObjectDetector::dropObjectCallback(const homodeus_msgs::HDBoundingBox& boundingBoxMsg) {
  ROS_INFO_STREAM("ObjectDeopMSG RECEIVED");
  std::vector<moveit_msgs::CollisionObject> obstacles_copy = _obstacle_list;

  publishDropPose(boundingBoxMsg.id, obstacles_copy);
}

double ObjectDetector::distanceBetweenPositions(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  pcl::PointXYZ pcl_point1 = pcl::PointXYZ(p1.x, p1.y, p1.z);
  pcl::PointXYZ pcl_point2 = pcl::PointXYZ(p2.x, p2.y, p2.z);

  return euclideanDistance(pcl_point1, pcl_point2);
}

visualization_msgs::Marker ObjectDetector::createBoundingBox(std_msgs::Header header, geometry_msgs::Pose pose, std_msgs::ColorRGBA color, pcl::PointXYZ max_point_OBB, pcl::PointXYZ min_point_OBB,  int id, bool isPlan){

  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header =header;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.pose = pose;
  marker.scale.x = 0.005f;
  marker.color = color;
  marker.lifetime = ros::Duration(5);


  float max_x = max_point_OBB.x;
  float max_y = max_point_OBB.y;
  float max_z = max_point_OBB.z;

  float min_x = min_point_OBB.x;
  float min_y = min_point_OBB.y;
  float min_z = min_point_OBB.z;

  float width = max_x - min_x;
  float length = max_y - min_y;
  float height = max_z - min_z;
  if (isPlan) {
    width += 0.05;
    length += 0.05;
  }

  std::vector<Eigen::Vector3f> local_points = {
      {max_x, max_y, max_z}, {max_x - width, max_y, max_z}, {max_x, max_y - length, max_z}, {max_x - width, max_y - length, max_z},
      {max_x, max_y, max_z - height}, {max_x - width, max_y, max_z - height}, {max_x, max_y - length, max_z - height}, {max_x - width, max_y - length, max_z - height}
  };

  for (size_t i = 0; i < local_points.size(); ++i) {
      for (size_t j = i + 1; j < local_points.size(); ++j) {
          if (true) {
              marker.points.push_back(vectorToPoint(local_points[i]));
              marker.points.push_back(vectorToPoint(local_points[j]));
          }
      }
  }

  return marker;
}

moveit_msgs::CollisionObject ObjectDetector::createObstacles(float Longueur, float largeur, float Hauteur, pcl::PointXYZ position_OBB, geometry_msgs::Quaternion quaternion, std::string id){

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = largeur;
  primitive.dimensions[1] = Longueur;
  primitive.dimensions[2] = Hauteur;

  geometry_msgs::Pose box_pose;
  box_pose.position.x = position_OBB.x;
  box_pose.position.y = position_OBB.y;
  box_pose.position.z = position_OBB.z;

  box_pose.orientation.x = quaternion.x;
  box_pose.orientation.y = quaternion.y;
  box_pose.orientation.z = quaternion.z;
  box_pose.orientation.w = quaternion.w;


  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.APPEND;
  return collision_object;
}

geometry_msgs::Point ObjectDetector::vectorToPoint(const Eigen::Vector3f& point) {
    geometry_msgs::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    return p;
}

void ObjectDetector::start()
{
  _cloudSub = _nh.subscribe("cloud", 1, &ObjectDetector::cloudCallback, this);
  _planCloudSub = _nh.subscribe("plane_cloud", 1, &ObjectDetector::planeCloudCallback, this);
  _objectPickSub = _nh.subscribe("/Homodeus/Behaviour/Take/Request", 1, &ObjectDetector::pickObjectCallback, this);
  _objectDropSub = _nh.subscribe("/Homodeus/Behaviour/Drop/Request", 1, &ObjectDetector::dropObjectCallback, this);
}

void ObjectDetector::stop()
{
  _cloudSub.shutdown();
  _planCloudSub.shutdown();
  _objectPickSub.shutdown();
  _objectDropSub.shutdown();
}

void ObjectDetector::run()
{
  ros::Rate loopRate(_rate);

  double halfPeriod = 0.5*1.0/_rate;

  while ( ros::ok() )
  {
    bool anySubscriber = _prehension_pick_pose.getNumSubscribers() > 0 ||
                         _prehension_drop_pose.getNumSubscribers() > 0 ||
                         _objectVisualisationPosePub.getNumSubscribers() > 0 ||
                         _planeVisualisationMarkerPub.getNumSubscribers() > 0 ||
                         _objectVisualisationMarkerPub.getNumSubscribers() > 0;


    if (anySubscriber )
    {
      ROS_INFO("Enabling node because there are subscribers");
      start();
    }
    else if (!anySubscriber )
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
