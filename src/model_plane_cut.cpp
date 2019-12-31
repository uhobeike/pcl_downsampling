#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>

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
seg.setDistanceThreshold (0.01);//RANSACの最小二乗法の許容誤差範囲
seg.setMaxIterations(50);
seg.setProbability(0.95);

pcl::ExtractIndices<pcl::PointXYZRGB> extract;
pcl::PointCloud<pcl::PointXYZRGB> cloud_output;
extract.setInputCloud(cloud.makeShared());
seg.segment (*inliers, *coefficients);
extract.setIndices(inliers);
extract.setNegative(true);//trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
extract.filter(cloud_output);

pub.publish(cloud_output);
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
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("model_plane_cut", 1);
  // Spin
  ros::spin ();
}