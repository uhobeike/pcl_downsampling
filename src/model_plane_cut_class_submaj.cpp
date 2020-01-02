#include <boost/bind.hpp>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <functional>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// PCL specific includes
#include "pcl_ros/point_cloud.h"
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

class PointCloudTransform{
	private:
          ros::NodeHandle nh;
		      ros::NodeHandle nhPrivate;
		      ros::Subscriber sub;
          ros::Publisher pub;
		      std::string target;
  public:
          PointCloudTransform();
          void tf_broadcast(const std::string frame_id);
          void Callback(const sensor_msgs::PointCloud2ConstPtr& msg, const std::string &frame_id);
};
PointCloudTransform::PointCloudTransform()
    : nhPrivate("~")
{
  std::string frame_id;
	sub = nh.subscribe<std::string>("input", 1, &PointCloudTransform::Callback,  boost::bind(&PointCloudTransform::Callback,_2,std::ref(frame_id)));
	pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("model_plane_cut", 1);
	nhPrivate.getParam("target", target);
};

void PointCloudTransform::tf_broadcast(const std::string &frame_id){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_depth_optical_frame";
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform.translation.x = 2.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}
void PointCloudTransform::Callback(const sensor_msgs::PointCloud2ConstPtr &input, const std::string &frame_id)
{
  const static std::string EXAMPLE_FRAME_ID = "example_frame";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>()); 
  pcl::fromROSMsg (*input,*cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;

  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setInputCloud(cloud);
  seg.setModelType (pcl::SACMODEL_PLANE);//検出するモデルのタイプを指定
  seg.setMethodType (pcl::SAC_RANSAC);//検出に使用する方法を指定
  double DistanceThreshold_param;
  ros::param::get("DistanceThreshold_param", DistanceThreshold_param);
  seg.setDistanceThreshold (0.01);//RANSACの最小二乗法の許容誤差範囲
  seg.setMaxIterations(50);
  seg.setProbability(0.95);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZRGB>());
  extract.setInputCloud(cloud);
  seg.segment (*inliers, *coefficients);
  extract.setIndices(inliers);
  extract.setNegative(true);//trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
  extract.filter(*cloud_filter);


  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
  tree->setInputCloud (cloud_filter);

  
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filter);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract (cluster_indices);
  //pub.publish(cloud_output);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>());
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>());
    *cloud_output += *cloud_cluster;
  }
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_output, output);
  output.header.frame_id = frame_id;
  pub.publish(output);
  
  
  PointCloudTransform::tf_broadcast(EXAMPLE_FRAME_ID);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "model_plane_cut");
	
	PointCloudTransform transform;
	ros::spin();
}
