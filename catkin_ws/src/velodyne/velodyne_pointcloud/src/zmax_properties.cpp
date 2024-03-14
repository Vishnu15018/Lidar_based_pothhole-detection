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
#include <pcl/segmentation/sac_segmentation.h>
#include <velodyne_pointcloud/organized_cloudXYZIR.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/point_types.h>

// 
int count=1;
ros::Publisher pub_;
ros::Publisher pub_plane;
// static double distance;
#define BLUE 0x0000FF
#define RED 0xFF0000
#define WHITE 0xFFFFFF


// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;
    float euclideanDistance(const pcl::PointXYZRGB& point, float max_z) {
    // Calculate the differences in x, y, and z coordinates
    float dx = point.x;
    float dy = point.y;
    float dz = point.z - max_z;

    // Calculate the Euclidean distance
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance;
}
std::ofstream logfile;
int depth_end =6;
// const int size_array = 11;

//  --- 
float thr1 =0.036;
float thr2=0.080;
float x_strip_initial = 0.0;
float severity_thresholds[] = {2.0, 4.0, 5.0, 6.0, 7.0};


double no_frame=0,gremoval_time=0;
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  { 
      std::clock_t start = std::clock();
    // Stop the clock
    logfile <<"Threshold thr1: "<<thr1<<"Threshold2 thr2"<<thr2<<std::endl;
    std::cout<<"inside callback"<<endl;
    ros::Time timestamp = msg->header.stamp;

    // Print the timestamp
    ROS_INFO("Received PointCloud2 message with timestamp: %f", timestamp.toSec());
    logfile <<"timestamp : "<<timestamp.toSec()<<std::endl;
// Define strip size globally
float x_strip = 10.0; // Example value
float y_strip = 2.0;  // Example value
float strip_size = 0.25; // Example value
    // Convert ROS message to PCL point cloud

        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
      
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 
    pcl::fromROSMsg(*msg, *cloud);
    cloud_filtered.resize(cloud->size());
    // Check if the cloud is empty
    if (cloud->empty()) {
        ROS_WARN("Received empty point cloud.");
        return;
    }

    // Filter the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto &point : *cloud) {
        if (point.x > 0 && point.x < x_strip &&
            point.y > -y_strip && point.y <y_strip &&
            point.z < -1.1) {
            pcl::PointXYZRGB pcl_point;
            pcl_point.x=point.x;
            pcl_point.y =point.y;
            pcl_point.z=point.z;
            pcl_point.r=255;
            pcl_point.g=255;
            pcl_point.b=255;
            filtered_cloud->points.push_back(pcl_point);
            
            
        }
    }

    // Determine array size
    int rows = ceil(x_strip / strip_size) ;
    int cols =ceil((2 * y_strip) / strip_size);
    std::vector<std::vector<float>> z_max_array(rows, std::vector<float>(cols, -std::numeric_limits<float>::infinity()));

    // Create 2D array
    std::vector<std::vector<pcl::PointIndices>> inliers_array(rows, std::vector<pcl::PointIndices>(cols));

    // Iterate through filtered cloud and fill the array
for (int i = 0; i < filtered_cloud->size(); ++i) {
    const auto& point = filtered_cloud->points[i];
    int row_index = static_cast<int>((point.x - x_strip_initial)/strip_size);
    int col_index = static_cast<int>((point.y + y_strip) / strip_size);
    inliers_array[row_index][col_index].indices.push_back(i); 
    z_max_array[row_index][col_index] = std::max(z_max_array[row_index][col_index], point.z);
    // Store the index of the point
}


    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            std::cout << "z_max_array[" << i << "][" << j << "]: " << z_max_array[i][j] << std::endl;
        }
    }  




    // Create array to store model coefficients
    int min_points_for_ransac =4;
    std::vector<std::vector<pcl::ModelCoefficients>> coefficients_array(rows, std::vector<pcl::ModelCoefficients>(cols));
    // 
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // Check if the number of points is sufficient for RANSAC
            if (inliers_array[i][j].indices.size() <= min_points_for_ransac) {
                coefficients_array[i][j].values.assign(4, 0); // Set parameters to (0, 0, 0, 0)
                continue; // Skip RANSAC for this grid cell
            }

            // Extract points from filtered cloud using inliers
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(filtered_cloud);
            extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers_array[i][j]));
            extract.filter(*extracted_cloud);

            // Fit plane to extracted points
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(300);
            seg.setDistanceThreshold(0.01);
            seg.setInputCloud(extracted_cloud);
            seg.segment(*inliers, *coefficients);

            // Store model coefficients in array
            coefficients_array[i][j] = *coefficients;
        }
    }
      
    // Print the indices in each grid and plane coefficients in each grid
    // for (int i = 0; i < rows; ++i) {
    //         for (int j = 0; j < cols; ++j) {
    //             std::cout << "Grid (" << i << ", " << j << ") - Indices: ";
    //             for (size_t k = 0; k < inliers_array[i][j].indices.size(); ++k) {
    //                 std::cout << inliers_array[i][j].indices[k] << " ";
    //             }
                
    //             std::cout<<",Count "<<inliers_array[i][j].indices.size();
    //             std::cout << std::endl;
    //             std::cout << "Plane coefficients for grid (" << i << ", " << j << "): ";
    //             const pcl::ModelCoefficients& coefficients = coefficients_array[i][j];
    //             std::cout << "a: " << coefficients.values[0] << ", "
    //                       << "b: " << coefficients.values[1] << ", "
    //                       << "c: " << coefficients.values[2] << ", "
    //                       << "d: " << coefficients.values[3] << std::endl;
    //         }
    //     }
        
        

//   distance finding using norm of the plane.

//   Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    // float plane_d = coefficients->values[3];
    std::vector<float> label_count(depth_end,0);




for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
    const auto& point = filtered_cloud->points[i];
    int row_index = static_cast<int>((point.x - x_strip_initial) / strip_size);
    int col_index = static_cast<int>((point.y + y_strip) / strip_size);

    // const pcl::ModelCoefficients& coefficients = coefficients_array[row_index][col_index];

    // Eigen::Vector3f plane_normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    // Eigen::Vector3f point_vector(point.x, point.y, point.z);

    // float distance = euclideanDistance(point,z_max_array[row_index][col_index]);
    float distance = std::abs(point.z - z_max_array[row_index][col_index]);

    // if (coefficients.values[0] != 0 || coefficients.values[1] != 0 || coefficients.values[2] != 0) {
    //     distance = fabs(plane_normal.dot(point_vector) + coefficients.values[3]) / plane_normal.norm();
    // }

    int label = 0;
    std::cout << "distance: of point : " << i <<" "<<distance << std::endl;
    for (int j = 0; j < sizeof(severity_thresholds) / sizeof(severity_thresholds[0]); ++j) {
        if (distance * 100 >= severity_thresholds[j]) {
            label = j + 1; // Labels start from 1
        }
    }

    label_count[label]= label_count[label] + 1;
    float color_ratio = label / 5.0f; // Assuming distance ranges from 0 to 5
    // Modify point cloud colors based on label
    // filtered_cloud->points[i].r = static_cast<uint8_t>((1 - color_ratio) * ((BLUE >> 16) & 0xFF) + color_ratio * ((RED >> 16) & 0xFF));
    // filtered_cloud->points[i].g = static_cast<uint8_t>((1 - color_ratio) * ((BLUE >> 8) & 0xFF) + color_ratio * ((RED >> 8) & 0xFF));
    // filtered_cloud->points[i].b = static_cast<uint8_t>((1 - color_ratio) * (BLUE & 0xFF) + color_ratio * (RED & 0xFF));
    
    if(label == 1){
        filtered_cloud->points[i].r =0;
        filtered_cloud->points[i].g =255;
        filtered_cloud->points[i].b=0;
    }
    if(label == 2){
        filtered_cloud->points[i].r =0;
        filtered_cloud->points[i].g =0;
        filtered_cloud->points[i].b=255;
    }
    if(label==3){
        filtered_cloud->points[i].r =255;
        filtered_cloud->points[i].g =255;
        filtered_cloud->points[i].b=0;
    }
    if(label==4){
        filtered_cloud->points[i].r =255;
        filtered_cloud->points[i].g =165;
        filtered_cloud->points[i].b=0;
    }
    if(label==5){
        filtered_cloud->points[i].r =255;
        filtered_cloud->points[i].g =0;
        filtered_cloud->points[i].b=0;
    }
    // std::cout<<"label: "<<label<<"R: "<<filtered_cloud->points[i].r<<" g: "<<filtered_cloud->points[i].g<<" b: "<<filtered_cloud->points[i].b<<std::endl;
        // filtered_cloud->points[i].r=255;
    // filtered_cloud->points[i].g=0;
    // filtered_cloud->points[i].r=0;

}

// printing the labels
   for(int i=0;i<depth_end;i++){
       std::cout<<"label : "<< i<< " count "<<label_count[i]<<endl;
   }

        
    
    // for(int i=0 ;i<distances.size();i++){
    //     std::cout<<"distance index ; "<<i<<" is "<<distances[i]<<endl;
    // }
    std::cout<<endl;
    int size_filtered = filtered_cloud->size();
    int k=0;

    //  labelling the points 
    // for(int i=0;filtered_cloud->points.size();i++){
    //     pcl::PointXYZRGB pcl_point;
    //  std::cout<<"Enterd in to the distances "<<endl;
    //        pcl_point.x=cloud->points[i].x;
    //        pcl_point.y=cloud->points[i].y;
    //        pcl_point.z=cloud->points[i].z;
    //     if(cloud->points[i].x>0 && cloud->points[i].x<x_strip && cloud->points[i].y> -y_strip && cloud->points[i].y<y_strip && cloud->points[i].z < -1.1){
                    
    //         float distance = distances[k];
    //         int label = 0;
    //         std::cout<<"distance:"<<distance<<std::endl;
    //         for (int j = 0; j < sizeof(severity_thresholds) / sizeof(severity_thresholds[0]); ++j) {
    //             if (distance*100 >= severity_thresholds[j]) {
    //                 label = j + 1; // Labels start from 1
    //             }
    //         }
    //         float color_ratio = label/ 5.0f; // Assuming distance ranges from 0 to 5
    //             pcl_point.r = static_cast<uint8_t>((1 - color_ratio) * ((BLUE >> 16) & 0xFF) + color_ratio * ((RED >> 16) & 0xFF));
    //             pcl_point.g = static_cast<uint8_t>((1 - color_ratio) * ((BLUE >> 8) & 0xFF) + color_ratio * ((RED >> 8) & 0xFF));
    //             pcl_point.b = static_cast<uint8_t>((1 - color_ratio) * (BLUE & 0xFF) + color_ratio * (RED & 0xFF));
    //             k++;
    //     }
    //     else{
    //        pcl_point.r=255;
    //        pcl_point.g=255;
    //        pcl_point.b=255;
    //     }
    //     cloud_filtered.push_back(pcl_point);
    // }
    
    // Convert the filtered cloud with labels back to ROS message
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    // Publish the labeled point cloud
    output_msg.header = msg->header;
    pub_.publish(output_msg);


    std::clock_t end = std::clock();

    // Compute the elapsed time
    double elapsed_time = static_cast<double>(end - start) / CLOCKS_PER_SEC;


    // clearing the array 


    
    // Output the elapsed time
    logfile<<"Execution time: " << elapsed_time << " seconds" << std::endl;
    std::cout << "Execution time: " << elapsed_time << " seconds" << std::endl;

  }

  
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "path_hole_detection");
  ros::NodeHandle nh_ground_removal_new;
  double distance;
  nh_ground_removal_new.getParam("/ground_removal_new/distance_threshold",distance);
  // std::cout<<"Value of distance "<<distance<<"\n";
  //n.getParam("/main/x1",x1);
  std::cout<<"Hai"<<endl;
  ros::Subscriber sub_ = nh_ground_removal_new.subscribe("/velodyne_points", 1, callback);
  int x=1;
  std::string m1 ="Row_wise";
  std::string filename = "/home/lidar/Desktop/vishnu/output_txts/log_file_path_hole_" +m1 + std::to_string(thr1) + "_" + std::to_string(thr2)+ "_"+std::to_string(x) + ".txt";
  logfile.open(filename);
    if (!logfile.is_open()) {
        ROS_ERROR("Failed to open log file");
        return 1;
    }
  pub_ = nh_ground_removal_new.advertise<sensor_msgs::PointCloud2> ("/test_topic_zmax", 1);
  //pub_ = nh_ground_removal_new.advertise<PointCloud>("/published_topic", 1);
  pub_plane = nh_ground_removal_new.advertise<pcl_msgs::ModelCoefficients>("plane_model", 1);

  ros::spin();
   logfile.close();
  return 0;
}



