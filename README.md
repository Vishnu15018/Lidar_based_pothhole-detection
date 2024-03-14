# Path Hole Detection Algorithm
Code for the pathole detection is in **catkin_ws/src/velodyne/velodyne_pointcloud/src/path_hole_detection.cpp**
## Abstract

### Methodology:

1. **Grid Creation:**
   - Initially, a grid is created from x=0 to 10m and y=-2 to +2, with a strip size of 0.25.
   - Total number of rows: ceil(diff_x / strip_size)
   - Total number of columns: ceil(diff_y / strip_size)
   - Each grid cell is defined by the formula:
     ```
     row = floor(x / strip_size)
     col = floor((y + y_strip) / strip_size)
     ```
     where y_strip = 2.

2. **Assign Lidar Points:**
   - Lidar points are assigned to each grid using the following formulae:
     ```
     diff_x = 10
     diff_y = 4
     ```

3. **Z Average Calculation:**
   - Calculate the average z-value in each grid.

4. **Path Hole Detection:**
   - To identify path holes, compare each grid cell (`grid[i][j]`) with the average of its adjacent cells:
     ```
     avg(grid[i-1][j], grid[i+1][j], grid[i][j-1], grid[i][j+1])
     ```
   - If the difference falls within the threshold, it is classified as a pothole.
   
   - Threshold:
     ```
     (0.036, 0.080)
     ```

## Running the algorithm 
1. create a catkin_workspace and git clone the necessary libraries from like pcl from **http://wiki.ros.org/pcl_ros**, velodyne, and ros1 **( http://www.ros.org/wiki/velodyne)**.
2. Download the pothhole detection cpp file and store that file in velodyne_pointcloud/src libarary.
3. run the file using using rosrun command.

## results

1. results of pathhole detection algorithm.
<!  ![Result Image 1](results/paper_5_d2_25.49_cropped.png)
*Image reference and point cloud with speed bump highlighted * >

     

