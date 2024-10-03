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

// ROS headers
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

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
#include <homodeus_msgs/HDPose.h>
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

  void publishHDPose(const geometry_msgs::Pose& pose,
                      const homodeus_msgs::DesireID& desireID);

  void publishPosestamped(const geometry_msgs::Pose& pose,
                      const std_msgs::Header& header);

  void selectObject(geometry_msgs::Point pointBoundingBox, std::vector<geometry_msgs::Pose> pose_list);
  void objectDetectionCallback(const homodeus_msgs::ObjectDetection& objectDetectionMsg);
  double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
 
  void start();
  void stop();

  ros::NodeHandle& _nh, _pnh;
  ros::CallbackQueue _cbQueue;
  bool _enabled;
  double _rate;

  // ROS interfaces
  ros::Subscriber _cloudSub;
  ros::Subscriber _objectDetectionSub;
  
  ros::Publisher  _HDPosePub;
  ros::Publisher  _objectVisualisationPosePub;
  ros::Publisher  _objectVisualisationMarkerPub;


  std::vector<geometry_msgs::Pose> _objets_pos_list;
};


ObjectDetector::ObjectDetector(ros::NodeHandle& nh,
                                   ros::NodeHandle& pnh):
  _nh(nh),
  _pnh(pnh),
  _enabled(false),
  _rate(5.0)
{
  _nh.setCallbackQueue(&_cbQueue);

  pnh.param<double>("rate", _rate, _rate);

  _HDPosePub   = _pnh.advertise<homodeus_msgs::HDPose>("hd_pose", 1);
  _objectVisualisationPosePub   = _pnh.advertise< geometry_msgs::PoseStamped >("object_pose", 1);
  _objectVisualisationMarkerPub = _pnh.advertise<visualization_msgs::MarkerArray>("object_marker", 1);
}

ObjectDetector::~ObjectDetector()
{

}

void ObjectDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
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

  std::cout << "Nombre dobjets taille: " << taille << std::endl;
  
  if ( _objectVisualisationMarkerPub.getNumSubscribers() > 0 )
  {
    _objectVisualisationMarkerPub.publish(marker_array);
  }
}

void ObjectDetector::publishHDPose(const geometry_msgs::Pose& pose, const homodeus_msgs::DesireID& desireID)
{
  if ( _HDPosePub.getNumSubscribers() > 0)
  {
    homodeus_msgs::HDPose hd_pos_msg;
    hd_pos_msg.id = desireID;
    hd_pos_msg.pose = pose;
    _HDPosePub.publish(hd_pos_msg);
  }
}

void ObjectDetector::publishPosestamped(const geometry_msgs::Pose& pose, const std_msgs::Header& header)
{
  geometry_msgs::PoseStamped posestamped;
  posestamped.pose = pose;
  posestamped.header = header;

  _objectVisualisationPosePub.publish(posestamped); 
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

  // Select Object
  double small_dist = 999999999; 

  geometry_msgs::Pose pose_to_grasp;
  
  for (geometry_msgs::Pose pose : _objets_pos_list){
    double dist = calculateDistance(pose.position, point_in_base_link.point);
    if (dist < small_dist) {
      small_dist = dist;
      pose_to_grasp = pose;
    }
  }

  // TODO : Changer le msg homodeus_msgs::ObjectDetection Pour avoir desireID
  homodeus_msgs::DesireID desireID;
  desireID.desire_id = 0;

  publishHDPose(pose_to_grasp, desireID);
  publishPosestamped(pose_to_grasp, objectDetectionMsg.header);
}

double ObjectDetector::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  pcl::PointXYZ pcl_point1 = pcl::PointXYZ(p1.x, p1.y, p1.z);
  pcl::PointXYZ pcl_point2 = pcl::PointXYZ(p2.x, p2.y, p2.z);

  return euclideanDistance(pcl_point1, pcl_point2);
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
    bool anySubscriber = _HDPosePub.getNumSubscribers() > 0 ||
                         _objectVisualisationPosePub.getNumSubscribers() > 0 ||
                         _objectVisualisationMarkerPub.getNumSubscribers() > 0;


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
