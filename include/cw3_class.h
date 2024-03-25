/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW3_CLASS_H_
#define CW3_CLASS_H_

// system includes
#include <ros/ros.h>

// include services from the spawner package - we will be responding to these
#include "cw3_world_spawner/Task1Service.h"
#include "cw3_world_spawner/Task2Service.h"
#include "cw3_world_spawner/Task3Service.h"

// // include any services created in this package
// #include "cw3_team_x/example.h"
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
// testing
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/segmentation/region_growing.h>
class cw3
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw3(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw3_world_spawner::Task1Service::Request &request,
    cw3_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw3_world_spawner::Task2Service::Request &request,
    cw3_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw3_world_spawner::Task3Service::Request &request,
    cw3_world_spawner::Task3Service::Response &response);

  // Basic functions
  bool moveArm(geometry_msgs::Pose target_pose);
  geometry_msgs::TransformStamped tfTransform(std::string t_frame, std::string s_frame);
  geometry_msgs::PointStamped pointInWorld(float x, float y, float z, std::string s_frame, std::string t_frame);
  void calOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out); 
  geometry_msgs::Quaternion getOrientation();
  void add_plane();
  void addCollision(std::string object_name, geometry_msgs::Point centre, 
                             geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation);


  // Task 1
  bool task1Move(geometry_msgs::PointStamped object_point, 
    geometry_msgs::PointStamped goal_point, std::string shape_type);

  bool moveToTargetPoint(geometry_msgs::PointStamped target_point);

  bool moveGripper(float width);

  bool pickObj(geometry_msgs::Pose pose);

  bool dropObj(geometry_msgs::Pose pose);

  geometry_msgs::Pose getPose(geometry_msgs::PointStamped target_point);

  void extractObjectsClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr color_cloud, std::vector<pcl::PointIndices>& cluster_indices, 
                                float tolerance, int min_cluster_size, int max_cluster_size);

  void imageCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);
  void tfCallback();
  // Task 2



  // Task 3


  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  // basic variables
  ros::Subscriber sub_task; // subscirber for pointcloud
  tf::TransformListener tf_listener;
  /** \brief Parameters to define the pick operation */
  double length = 0.04;
  double height = 0.04;
  double basket_h = 0.05;
  double gripper_open_ = 0.08;
  double gripper_closed_ = 0.02;
  double z_offset_obj = 0.115;
  double pi_ = 3.14159;
  double angle_offset_ = 3.14159 / 4.0;
  double angle = 0;
  double angle_new = 0;

  // double approach_distance_basket = 0.205;
  double approach_distance_basket = 0.155;
  double approach_distance_obj = 0.125;

  /** \brief MoveIt interface to move groups to seperate the arm,
    * these are defined in urdf. */
  std::vector<moveit::planning_interface::MoveGroupInterface> arm_groups;
  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
    * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // filter
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;

  geometry_msgs::PointStamped object_point;
  geometry_msgs::PointStamped goal_point;
  std::string shape_type;
  // variables for task 1
  bool sub_flag_task1 = false;
  std::string hand_frame = "panda_hand";
  std::string target_frame = "panda_link0";
  std::string camera_frame = "color";
  geometry_msgs::PointStamped point_in, point_out;
  double min_y_p;
  double max_y_p;
  double min_x_p;
  double max_x_p;
  geometry_msgs::TransformStamped transform_matrix;

  geometry_msgs::Vector3 hand_position;
  double translation_hand_camera_x;
  double translation_hand_camera_y;
  double translation_hand_camera_z;
  geometry_msgs::Quaternion orientation;
};

#endif // end of include guard for cw3_CLASS_H_

