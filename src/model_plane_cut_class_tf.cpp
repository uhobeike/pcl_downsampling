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
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

class PointCloudTransform{
	private:
          ros::NodeHandle nh;
		      ros::NodeHandle nhPrivate;
		      ros::Subscriber sub;
          ros::Publisher pub;
          tf::TransformListener tf_listener_;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tranformed_;
          pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
		      sensor_msgs::PointCloud pc_;
		      std::string target_frame_;
  public:
          PointCloudTransform();
          void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        
};
PointCloudTransform::PointCloudTransform()
  :nh(),nhPrivate("~")
{
	sub = nh.subscribe("input", 1, &PointCloudTransform::Callback, this);
	pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("model_plane_cut", 1);
  nhPrivate.param("target_frame", target_frame_, std::string(""));
};
void PointCloudTransform::Callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
  pcl::fromROSMsg (*input, input_cloud);
  //pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_src = input;
  std::cout << "uho" << std::endl;
	try{
    std::string frame_id = input->header.frame_id;
    //pcl::PointCloud2::ConstPtr cloud_src = input;
    std::cout << "uho" << std::endl;
    if (target_frame_.empty() == false)
    {
      frame_id = target_frame_;
      if (pcl_ros::transformPointCloud(target_frame_, input_cloud ,*cloud_tranformed_, tf_listener_) == false)
      {
        ROS_ERROR("Failed pcl_ros::transformPointCloud. target_frame = %s",
                  target_frame_.c_str());
        return;
      }
      pub.publish(cloud_tranformed_);
      std::cout << "uho" << std::endl;
    }
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
  /*
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
  */
  //pub.publish(input_cloud);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "model_plane_cut");
	
	PointCloudTransform transform;
	ros::spin();
}
