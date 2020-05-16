#include <iostream>
#include <string>
#include <stdlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
int main (int argc, char** argv)
{ 
  int roop[] = {1,2};
  
  std::string HOME = getenv("HOME");
  
  for(int i : roop)
  {	
	  //点群用変数
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_re (new pcl::PointCloud<pcl::PointXYZ>);

	  //ファイルのインプット
	  std::string pcd_path_input = HOME + "/Downloads/output_" + std::to_string(i) + ".bag_points.pcd"; 
	  pcl::io::loadPCDFile (pcd_path_input, *cloud_re);
          
	  std::cout << "////////////////////////" << std::endl;
  	  std::cout << "output_" + std::to_string(i) + ".bag_points.pcd" << std::endl << "downsampling_start" << std::endl << std::endl;
  
	  //ダウンサンプリング用変数宣言
	  pcl::VoxelGrid<pcl::PointXYZ> vg;
	  //ダウンサンプリング用の点群格納変数
	  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>());
	  
	  //ダウンサンプリングするためにインプットした点群をセット
	  vg.setInputCloud(cloud_re);
	  //ダウンサンプリング用しきい値
	  vg.setLeafSize (0.1f, 0.1f, 0.1f);
	  //ダウンサンプリング用変数に格納する
	  vg.filter(*tmp);

	  //ダウンサンプリングした点群をpcdデータとして保存
	  std::string pcd_path_output = HOME + "/Downloads/output_" + std::to_string(i) + "_dow.bag_points.pcd";
          pcl::io::savePCDFileASCII(pcd_path_output, *tmp);

	  std::cout << "output_" + std::to_string(i) + "_dow.bag_points.pcd" << std::endl << "downsampling_finish" << std::endl;
	  std::cout << "////////////////////////" << std::endl;

  }

  return (0);
}
