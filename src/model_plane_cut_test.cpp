#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){

pcl::PointCloud<pcl::PointXYZRGB> cloud;
pcl::fromROSMsg (*input,cloud);

pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

pcl::SACSegmentation<pcl::PointXYZRGB> seg;

seg.setOptimizeCoefficients (true);
// Mandatory
seg.setInputCloud(cloud.makeShared());
seg.setModelType (pcl::SACMODEL_PLANE);//検出するモデルのタイプを指定
seg.setMethodType (pcl::SAC_RANSAC);//検出に使用する方法を指定
double DistanceThreshold_param;
ros::param::get("DistanceThreshold_param", DistanceThreshold_param);
seg.setDistanceThreshold (0.1);//RANSACの最小二乗法の許容誤差範囲
seg.setMaxIterations(50);
seg.setProbability(0.95);

pcl::ExtractIndices<pcl::PointXYZRGB> extract;
pcl::PointCloud<pcl::PointXYZRGB> cloud_output;
extract.setInputCloud(cloud.makeShared());
seg.segment (*inliers, *coefficients);
extract.setIndices(inliers);
extract.setNegative(true);//trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
extract.filter(cloud_output);

sensor_msgs::PointCloud2 output;
pcl::toROSMsg(cloud_output, output);

pub.publish(output);


}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "model_plane_cut");
  ros::NodeHandle nh;
  // Set ROS param
  ros::param::set("DistanceThreshold_param", 0.01);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2>("model_plane_cut", 1);
  // Spin
  ros::spin ();
}