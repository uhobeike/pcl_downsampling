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

void tf_broadcast(const std::string frame_id){
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


void euclideanClusterExtraction (const sensor_msgs::PointCloud2ConstPtr& input, const std::string frame_id){

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
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>());
extract.setInputCloud(cloud.makeShared());
seg.segment (*inliers, *coefficients);
extract.setIndices(inliers);
extract.setNegative(true);//trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
extract.filter(*cloud_out);

// Creating the KdTree object for the search method of the extraction
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
tree->setInputCloud (cloud_out);

std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
ec.setClusterTolerance (0.2); // 2cm
ec.setMinClusterSize (1);
ec.setMaxClusterSize (25000);
ec.setSearchMethod (tree);
ec.setInputCloud (cloud_out);
ec.extract (cluster_indices);

int cluster_i=0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>());
for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>());
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_out->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    *cloud_output += *cloud_cluster;


}


sensor_msgs::PointCloud2 output;
pcl::toROSMsg(*cloud_output, output);
output.header.frame_id = frame_id;
pub.publish(output);


}
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    static std::string EXAMPLE_FRAME_ID = "example_frame";

    euclideanClusterExtraction(input, EXAMPLE_FRAME_ID);

    // to shift positions of rendering point clouds
    tf_broadcast(EXAMPLE_FRAME_ID);
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