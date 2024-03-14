//  This is for speed bump detection.


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/registration/icp.h>
#include <math.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <octomap/octomap.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/PointIndices.h>
#include <ctime>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include<ctime>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <velodyne_pointcloud/organized_cloudXYZIR.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/point_types.h>


int count=1;
ros::Publisher pub_;
//static double distance;
ros::Publisher distance_pub;

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;
std::ofstream logfile;

// array size for storing strip coordinates 




//  Actual Thresholds for speed bump
float thr1 = 0.0700;
float thr2 = 0.20;

// strip_size
float strip_size=1.0;  
        std::ofstream outputFile; 
 // field of vie for analyzing the speed bump 
float y_strip=1.0;   
float x_strip =12.0;
const int size_array = ceil(x_strip/strip_size);     // we can use floor((x_strip/strip_size)+1)
//  this tells which strips we are analyzing in the direction of x
float buffer_end =0;                  // size_array - bufferend 
float buffer_start = 0;


//  to highlight the speed bump , y coorinates
      float y_strip1 = -4.2;
      float y_strip2 = 3;


//  ground height 

  float ground_height = -1.48;
  double no_frame=0,gremoval_time=0;




  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {     std::clock_t start = std::clock();
    // Stop the clock

    std::vector<std::vector<float>> z_values(size_array);
// std::array<std::vector<float>, size_array> z_values;
    std::vector<float> avg_z(size_array,0);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud1;
    pcl::fromROSMsg (*msg, cloud1);

  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_out (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
//   //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  *cloud=cloud1;
  // PointCloud::ConstPtr *cloud_out =boost::make_shared<const PointCloud>(*cloud_out) ;
  //  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // std::cout<<"Fstatiscial filter"<<endl;
    // pcl::StatisticalOutlierRemoval<velodyne_pointcloud::PointXYZIR> sor;
    // sor.setInputCloud(*cloud_out);
    // sor.setMeanK(50); // Adjust as needed
    // sor.setStddevMulThresh(1.0); 
    // sor.filter(*cloud);
    
  ros::Time::init();
  ros::Time begin= ros::Time::now();
   std::cout<<begin<<std::endl;

   pcl::PointIndices::Ptr inliers(new pcl::PointIndices());


// float z_bin=0.1;
// float max_z=-2.0;
// float min_z=-1.1;
// const int No_bins = abs(max_z - min_z) / z_bin;
// std::vector<int> z_bin_count(No_bins,0);



float min=0;
for (int i=0 ; i < cloud->points.size ();++i){
    if(cloud->points[i].z<=min){
      min=cloud->points[i].z;
    }
    if (cloud->points[i].x>0 && (cloud->points[i].y >=-y_strip && cloud->points[i].y <= y_strip) && cloud->points[i].z <= -1.10 && cloud->points[i].z>=-2.00){
            int index= static_cast<int>((cloud->points[i].x)/strip_size) ;
            // std::cout<<"x: "<<cloud->points[i].x<<endl;
            // std::cout<<index<<endl;

            // z_bin
            // int bin_index=0;

            // float z=cloud->points[i].z;
            // if(z>=max_z && z<=min_z){
            //     bin_index=static_cast<int>(((abs(z)-abs(min_z))/z_bin));
            // }
            // z_bin_count[bin_index] = z_bin_count[bin_index]+1;
            // std::cout<<"Bin_Index: "<<bin_index<<std::endl;


           if (index>=0 and index<size_array){
                z_values[index].push_back(cloud->points[i].z);    
    }
    }
 }




ros::Time timestamp = msg->header.stamp;

logfile << " frame: " << no_frame << std::endl;
std::cout<< " frame: " << no_frame << std::endl;
logfile <<"Average_zvalues"<<std::endl;



for (int i = 0; i < size_array; ++i) {
        float sum = 0.0;
        for (const auto& z : z_values[i]) {
            sum +=z;
        }
        float average = z_values[i].empty() ? ground_height: sum / z_values[i].size();
        avg_z[i]=average;
        // ROS_INFO_STREAM("Average z value for points with x in range " << (i) << " to " << (i +1) << ": " << average);
        // logfile << timestamp << "Average z value for points with x in range " << i << " to " << i+1 << ": " << average << std::endl;
        logfile<<average<<std::endl; 
        std::cout<<"index: "<<i<<"is"<<average<<std::endl;  
    }

    
logfile<<"Average_values end"<<std::endl;


// for(int i=0;i<size_array;++i){
//      std::cout<<avg_z[i]<<" ";
// }

std::cout<<endl;
//creating a pointcloud with rgb info
    for (auto& point_old : cloud->points)

    {  

            pcl::PointXYZRGB pcl_point;
            pcl_point.x = point_old.x;
            pcl_point.y = point_old.y;
            pcl_point.z = point_old.z;
        if (point_old.x >=0  and point_old.x < x_strip && point_old.y >= -y_strip && point_old.y <= y_strip && point_old.z<-1.10) {

            pcl_point.r = 0;
            pcl_point.g = 255;
            pcl_point.b = 0;
            
        }
        else{
            pcl_point.r = 255;
            pcl_point.g = 255;
            pcl_point.b = 255;
        }
        cloud_filtered.push_back(pcl_point);
    
        
    }


float n=0,n1=0,n2=0;
std::vector<float>avg_diff(size_array-1,0.0f);
logfile<<"avg_diff_start"<<std::endl;
// avg_z[size_array+1]=10;
std::cout<<"avg[size_array]"<<avg_z[size_array+1]<<endl;
for(int i=buffer_start;i<size_array-buffer_end -1 ;++i){
    float distance =-1;
   std::cout<<"average diff: "<<avg_z[i] - avg_z[i+1]<< "for "<<i<<" to "<<i+1<<endl;
  //  logfile << "average diff b/w: "<<i<< "to "<<i+1<<"is"<<avg_z[i] - avg_z[i+1]<<std::endl;
  //  logfile<<"index: "<<i<<std::endl;
   logfile<<avg_z[i] - avg_z[i+1]<<endl;
   avg_diff.push_back(avg_z[i] - avg_z[i+1]);
   if( (std::abs(avg_z[i] - avg_z[i+1]) >= thr1 ) && (std::abs(avg_z[i] - avg_z[i+1]) < thr2)){
         
         distance=i+1;
         std_msgs::Float32 distance_msg;
         distance_msg.data = distance;
         distance_pub.publish(distance_msg);
      std::cout<<"Index : "<< i <<endl;
      // logfile << "Index: Threshold statisfies"<<i<<std::endl;
      n=strip_size*i;
      n1=strip_size*(i+1);
      n2=strip_size*(i+2);

      float x1=(n+n1)/2;
      float x2= (n2+n1)/2;
      int r= 255;
      int g=0;
      int b=0;
      std::cout<<"r: "<<r<<"g: "<<g<<" b: "<<b<<endl;

      for (size_t j = 0; j < cloud_filtered.size(); ++j) {
          // Get the point's x coordinate
          if (cloud_filtered.points[j].x>0 && cloud_filtered.points[j].y> y_strip1 && cloud_filtered.points[j].y< y_strip2 && cloud_filtered.points[j].z <= -1.10){
            float x = cloud_filtered.points[j].x;
            
            // Check if the x coordinate lies between x1 and x2
            if (x >= x1 && x <= x2) {
                // Create a new point with XYZ coordinates
                // Set color of the point to red (255, 0, 0)
                // point.rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                cloud_filtered.points[j].r = r;
                cloud_filtered.points[j].g = g;
                cloud_filtered.points[j].b= b;
                // Push the colored point to the colored point cloud    
            }
           
            }
         }
}
}



logfile<<"end_average_distances"<<endl;





//   std::cout<<"average ground removal time: "<< (gremoval_time/no_frame)<<std::endl;
// std::cout << "Total no of frames: "<<no_frame<<std::endl;

sensor_msgs::PointCloud2 output_msg;
pcl::toROSMsg(cloud_filtered, output_msg);
output_msg.header = msg->header;
    // Publish the filtered point cloud
pub_.publish(output_msg);

    std::clock_t end = std::clock();

    // Compute the elapsed time
    double elapsed_time = static_cast<double>(end - start) / CLOCKS_PER_SEC;

    // Output the elapsed time
    std::cout << "Execution time: " << elapsed_time << " seconds" << std::endl;

// Open file in append mode
    outputFile << no_frame<< "," << elapsed_time *1000 << std::endl;
   
    no_frame++;


  }
 
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "ground_removal_ring");
  ros::NodeHandle nh_ground_removal_new;
  double distance;
  nh_ground_removal_new.getParam("/ground_removal_new/distance_threshold",distance);
  // std::cout<<"Value of distance "<<distance<<"\n";
  //n.getParam("/main/x1",x1);
  ros::Subscriber sub_ = nh_ground_removal_new.subscribe("/velodyne_points", 1, callback);
  distance_pub = nh_ground_removal_new.advertise<std_msgs::Float32>("/distance_to_speed_bump", 1);
  
  logfile.open("/home/lidar/Desktop/vishnu/log_file_test1.txt");
    if (!logfile.is_open()) {
        ROS_ERROR("Failed to open log file");
        return 1;
    }
  outputFile.open("/home/lidar/Desktop/vishnu/frame_time.csv");
  if (!outputFile.is_open()){
        ROS_ERROR("Failed to open frame_time_csv file");
        return 1;
  }
  outputFile << "frame,time" << std::endl;


  pub_ = nh_ground_removal_new.advertise<sensor_msgs::PointCloud2> ("/speed_bump", 1);
  //pub_ = nh_ground_removal_new.advertise<PointCloud>("/published_topic", 1);

  ros::spin();
   logfile.close();
   outputFile.close();

  return 0;
}



