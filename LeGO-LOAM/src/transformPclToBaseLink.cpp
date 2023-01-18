#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

// Declare a publisher
ros::Publisher pub;
std::string frame_id;
tf::TransformListener *listener_center_bumper_ptr;

void topic_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // Get the transform from base_link to lidar
  tf::StampedTransform transform;
  try {
    listener_center_bumper_ptr->lookupTransform(frame_id, msg->header.frame_id, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }

  // Convert to Eigen
  Eigen::Affine3d transform_matrix;
  tf::transformTFToEigen(tf::Transform(transform), transform_matrix);

  // Transform point cloud  
  sensor_msgs::PointCloud2 msg_pub;
  pcl_ros::transformPointCloud(transform_matrix.matrix().cast<float>(), *msg, msg_pub);
  msg_pub.header.frame_id = frame_id;  // Must set new frame manually

  pub.publish(msg_pub);
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"transform_cloud_nodelet");

  ros::NodeHandle nh("~");
  
  tf::TransformListener listener_center_bumper;   // atc/vehicle/center_bumper
  listener_center_bumper_ptr=&listener_center_bumper;

  // Get input params
  std::string pointcloud_to_transform;
  std::string pointcloud_transformed;
  nh.getParam("pointcloud_to_transform", pointcloud_to_transform);
  nh.getParam("frame_id", frame_id);
  nh.getParam("pointcloud_transformed", pointcloud_transformed);

  ROS_INFO("\nInput parameters:");
  ROS_INFO("pointcloud_to_transform: %s", pointcloud_to_transform.c_str());
  ROS_INFO("pointcloud_transformed: %s", pointcloud_transformed.c_str());
  ROS_INFO("frame_id: %s\n", frame_id.c_str());
  
  // Getting point cloud from launch file
  if (pointcloud_to_transform.empty()) {
    ROS_ERROR("Must set properly an input pointcloud");
    return -1;
  }
  
  // Getting reference frame to transform
  if (frame_id.empty()) {
    ROS_ERROR("Must set properly a frame_id to transform");
    return -1;
  }
  
  // Getting reference frame to transform
  if (pointcloud_transformed.empty()) {
    ROS_ERROR("Must set properly a topic to publish the transformed cloud");
    return -1;
  }
 
  ros::Subscriber sub = nh.subscribe(pointcloud_to_transform, 1, topic_callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_transformed, 1000);
   
  ros::spin();
  return 1;
}
