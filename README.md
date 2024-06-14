**rosbag_filter_project:** 
  - The rosbag_filter_project is a ROS node designed to process ROS bag files by filtering out and extracting specific sensor data topics. 
      > **This node aims to create a new ROS bag file named filtered_<original_file_name>.bag containing only the relevant data.**
  - The primary goal is to remove data when the vehicle is stationary, ensuring that only relevant data is preserved. 
  - This project aims to improve the quality of the data by eliminating noise and ensuring the integrity of the data collected from various sensors.

**Features:**
  - Node Setup: A ROS node developed in C++ to process ROS bag files.
  - Data Filtering: Filters messages from specific topics (/odom, /scan, /camera, /imu/data) and writes the filtered data to a new ROS bag file.
  - Motion Detection: Includes logic to check if the vehicle is moving based on odometry data.
  - Robust Error Handling: Implements error handling for file operations to ensure smooth execution.

**Enhancements (my next steps):**
  > Additional Filtering Criteria- Implement more sophisticated filtering based on additional criteria or sensor data analysis.
> 
  > GUI Integration- Develop a GUI tool for visualizing and managing the filtering process.

**Known Issues:**
  - BagIOException: Handling of large files and ensuring the correct paths for ROS bag files.

>
**Acknowledgements**
This project is developed as part of a larger effort to improve data quality for autonomous vehicle research, in collaboration with **Aaron Kingery and Dez Song**'s team.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
**If you prefer to clone this repository:** 

  **Installation/Prerequisites:**
    - ROS Noetic
    - C++11 or higher
    - Catkin build system

  *Clone the repository*:{
    git clone https://github.com/kaitlin-a/rosbag_filter_project.git
    } 

  *Navigate to catkin workspace and create a symbolic link to the project*:{
    cd ~/catkin_ws/src
    ln -s /path/to/rosbag_filter_project .
    }

  *Build the workspace*: {
    cd ~/catkin_ws
    catkin_make
    }

  *Source the setup file*: {
    source devel/setup.bash
    }

  *Usage*
    To run the node and process a ROS bag file: {
    rosrun rosbag_filter rosbag_filter_node /path/to/<rosbag_file>.bag
    }

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
This project is licensed under the MIT License. See the LICENSE file for details.
