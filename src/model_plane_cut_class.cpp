#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>

#include <tf/transform_listener.h>

class PointCloudTransform{
	private:
          ros::NodeHandle nh;
		      ros::NodeHandle nhPrivate;
		      ros::Subscriber sub;
           ros::Publisher pub;
		      sensor_msgs::PointCloud pc_;
		      std::string target;
  public:
          PointCloudTransform();
          void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
};
PointCloudTransform::PointCloudTransform()
    : nhPrivate("~")
{
	sub = nh.subscribe("input", 1, &PointCloudTransform::Callback, this);
	pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("model_plane_cut", 1);
	nhPrivate.getParam("target", target);
};
void PointCloudTransform::Callback(const sensor_msgs::PointCloud2ConstPtr &input)
{

  /*
	try{
		tflistener.waitForTransform(target, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
		tflistener.transformPointCloud(target, msg->header.stamp, pc_in, msg->header.frame_id, pc_trans);
		sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_out);
		pub.publish(pc2_out);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
  */
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

int main(int argc, char** argv){
	ros::init(argc, argv, "model_plane_cut");
	
	PointCloudTransform transform;
	ros::spin();
}

