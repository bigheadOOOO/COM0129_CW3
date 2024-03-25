/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

#include <cw3_class.h> // change to your team name here!

///////////////////////////////////////////////////////////////////////////////

cw3::cw3(ros::NodeHandle nh)
{
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw3::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw3::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw3::t3_callback, this);
  sub_task = nh_.subscribe("/r200/camera/depth_registered/points", 1, 
    &cw3::imageCallback, this);
  sub_flag_task1 = false;
  // define Orienation: 
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_+angle);
  tf2::Quaternion q_result = q_x180deg * q_object;
  orientation = tf2::toMsg(q_result);
  ROS_INFO("cw3 class initialised");
}

///////////////////////////////////////////////////////////////////////////////
////// Message type in Task1Service
//////// geometry_msgs/PointStamped object_point
//////// geometry_msgs/PointStamped goal_point
//////// string shape_type
//////// ---

bool
cw3::t1_callback(cw3_world_spawner::Task1Service::Request &request,
  cw3_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  
  // Collect message in the service
  object_point = request.object_point;
  goal_point = request.goal_point;
  shape_type = request.shape_type;
  // add the plane as a collision
  add_plane();
  // Execute movement 
  std::cout<<"===========object posi:"<<object_point.point.x<<", "<<object_point.point.y<<", "<<object_point.point.z<<std::endl;

  task1Move(object_point, goal_point, shape_type);  
  ROS_INFO("Finsh. ");
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t2_callback(cw3_world_spawner::Task2Service::Request &request,
  cw3_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t3_callback(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}

/////////////////////////////////////////////////////////////////////////////////
/* Basic Functions for All Tasks */
geometry_msgs::Pose cw3::getPose(geometry_msgs::PointStamped target_point)
{
  // PointStamped: std_msgs/Header header, geometry_msgs/Point point
  // geometry_msgs::Pose: geometry_msgs/Point position, geometry_msgs/Quaternion orientation
  //// geometry_msgs/Quaternion: float64 x, float64 y, float64 z, float64 w
  //// geometry_msgs/point: float64 x, float64 y, float64 z

  geometry_msgs::Point point = target_point.point; 
  geometry_msgs::Pose pose;
  pose.position.x = point.x - translation_hand_camera_x;
  pose.position.y = point.y - translation_hand_camera_y;
  pose.position.z = point.z; // - translation_hand_camera_z;

  pose.orientation = getOrientation();
  std::cout<<"======= after orientation:"<< pose.orientation.x<<","<<pose.orientation.y<<","<<pose.orientation.z<<std::endl;

  return pose;
}
/**
 * @brief Move the robot arm to a target pose.
 * 
 * @param target_pose The target pose to move the arm to.
 * @return True if the movement was successful, false otherwise.
 */
bool cw3::moveArm(geometry_msgs::Pose target_pose)
{
  // Step 1: Setup the target pose
  ROS_INFO("moveArm: Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // Step 2: Create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose);

  // 设置笛卡尔路径规划参数
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // 执行路径规划
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_ = trajectory;
  arm_group_.execute(my_plan);

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // bool success = (arm_group_.plan(my_plan) ==
  //   moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // // Step 3: Log the success of planning the path
  // ROS_INFO("Visualising plan %s", success ? "" : "FAILED");
  // if (not success) return success;
  // // Step 4: Execute the planned path
  // arm_group_.move();

  // Return the success status of the movement
  return true;
}

/**
 * @brief Move the robot gripper to a specified width.
 * 
 * @param width The desired width of the gripper.
 * @return True if the movement was successful, false otherwise.
 */
bool cw3::moveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_) 
    width = gripper_open_;
  if (width < gripper_closed_) 
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("moveGripper: Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // move the gripper joints
  hand_group_.move();

  return success;
}

bool cw3::pickObj(geometry_msgs::Pose pose)
{
  std::cout<<"````````````````````````````````PICK OBJECT: angle"<<angle<<std::endl;
  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose = pose;
  
  grasp_pose.position.z = (z_offset_obj+height/2);
  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose = grasp_pose;
  approach_pose.position.z = 0.5;
  
  // approach_pose.position.z += (z_offset_obj+approach_distance_obj+height);
  
  /* Now perform the pick */
  bool success = true;
  ROS_INFO("Begining pick operation");
  // move the arm above the object
  std::cout<<"/";
  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
  }
  // if (not success) 
  // {
  //   ROS_ERROR("Moving arm to grasping pose failed");
  // }
  
  double angle_old;
  
  for (int i = 0; i < 8; i++)
  {
    if(shape_type != "cross")
    {
      std::cout<<"||||||||: "<<min_y_p<<","<<max_y_p<<","<<min_y_p - max_y_p<<std::endl;
      std::cout<<"||||||||: "<<min_x_p<<","<<max_x_p<<","<<min_x_p - max_x_p<<","<<std::abs(min_x_p - max_x_p)<<std::endl;
      // ros::Duration(5).sleep();

      if ((std::abs(min_y_p - max_y_p) < 5.4*length && std::abs(min_x_p - max_x_p) < 5.4*length)&&(std::abs(min_y_p - max_y_p) > 4.95*length && std::abs(min_x_p - max_x_p) >4.95*length)) break;
    }else{
      std::cout<<"XXXXXXXXX: "<<min_y_p<<","<<max_y_p<<","<<min_y_p - max_y_p<<","<<(std::abs(min_y_p - max_y_p))<<std::endl;
      std::cout<<"XXXXXXXXX: "<<min_x_p<<","<<max_x_p<<","<<min_x_p - max_x_p<<","<<(std::abs(min_x_p - max_x_p))<<std::endl;
      // ros::Duration(5).sleep();

      if ((std::abs(min_y_p - max_y_p) > 4.86*length && std::abs(min_y_p - max_y_p) < 5.1*length) && (std::abs(min_x_p - max_x_p) > 4.86*length && std::abs(min_x_p - max_x_p) < 5.1*length)) break;
    }
    angle_old = angle;
    transform_matrix = tfTransform(target_frame, camera_frame);
    sub_flag_task1 = true;
    ros::Duration(0.1).sleep();
    while(sub_flag_task1)
    {
      ros::Duration(0.1).sleep();
    }
    
    approach_pose.orientation = getOrientation();
    if (std::abs(angle_new) < 1e-4) break;
    approach_pose.orientation = getOrientation();
  //   std::cout<<"/";
    success *= moveArm(approach_pose);
    if (not success) 
    {
      ROS_ERROR("Moving arm to grasping pose failed");
    }
    std::cout<<"*************** std::abs(angle-angle_old) > angle_offset_/8"<<std::abs(angle-angle_old)<<","<<angle<<std::endl;
  }
  
  
  // open the gripper  
  success *= moveGripper(gripper_open_);
  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
  }
  // approach to grasping pose
  
  // pose = getPose(object_point)
  
  std::cout<<"==grasp_pose_posi: "<<object_point.point.x<<","<<object_point.point.y<<","<<object_point.point.z<<std::endl;

  
  approach_pose.orientation = getOrientation();
  
  std::cout<<"===2 grasp_pose.orientation: "<<approach_pose.orientation.x<<","<<approach_pose.orientation.y<<","<<approach_pose.orientation.z<<std::endl;
  
  grasp_pose.orientation = approach_pose.orientation;
  approach_pose.position.x = object_point.point.x;
  approach_pose.position.y = object_point.point.y;
  grasp_pose.position.x = object_point.point.x;
  grasp_pose.position.y = object_point.point.y;
  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
  }
  success *= moveArm(grasp_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
  }
  // grasp!

  success *= moveGripper(gripper_closed_);
  if (not success) 
  {
    ROS_ERROR("Closing gripper to grasp failed");
  }
  // // retreat with object
  // // // get higher to avoid collision before dropping
  // approach_pose.position.z += height;
  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed: picked up, %f, %f", approach_pose.position.x, approach_pose.position.z);
  }
  return success;
}

bool cw3::dropObj(geometry_msgs::Pose pose)
{
  // move arm: go to the basket
  geometry_msgs::Pose drop_pose;
  drop_pose = pose;
  drop_pose.position.z += approach_distance_basket;
  drop_pose.position.z += z_offset_obj;

  // move arm to drop pose: plus cube_length to avoid the box collide with the basket
  int success = moveArm(drop_pose); 
  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed. move arm to drop pose");
    drop_pose.position.z += 0.05;
    // move arm to drop pose
    success = moveArm(drop_pose); 
  }

  // move gripper: drop
  success *= moveGripper(gripper_open_);

  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed. move gripper: drop");
    return false;
  }
  // leave the basket: get higher to avoid collide during movement
  drop_pose.position.z += basket_h;
  success *= moveArm(drop_pose);
  if (not success) 
  {
    ROS_ERROR("Leave basket failed. move gripper: drop"); 
  }else{
    ROS_INFO("== Drop done. ");
  }
  return success;
}

void cw3::extractObjectsClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr color_cloud, std::vector<pcl::PointIndices>& cluster_indices, 
                                float tolerance, int min_cluster_size, int max_cluster_size)
{
  /**
   * @brief Extract baskets clusters from the point cloud and store the cluster indices in cluster_indices
   * 
   */
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setInputCloud(color_cloud);
  ec.setClusterTolerance(tolerance);          // Set the tolerance for cluster segmentation
  ec.setMinClusterSize(min_cluster_size);     // Set the minimum number of points in a cluster
  ec.setMaxClusterSize(max_cluster_size);     // Set the maximum number of points in a cluster
  ec.extract(cluster_indices);                // Extracting and storing the indices of the clustered point cloud
  ROS_INFO("``````````````````cluster_indices %d", cluster_indices.size());
}


void cw3::imageCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{

  if (!sub_flag_task1) return; // only start to store the pointcloud message after task begins
  // Check if cloud_input_msg is valid
  if (cloud_input_msg->data.size() == 0) {
    ROS_ERROR("cloud_input_msg is empty.");
    return;
  }

  std::unique_ptr<pcl::PCLPointCloud2> cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*cloud_input_msg, *cloud);
  if (cloud->data.size() == 0) {
    ROS_ERROR("cloud is empty.");
    return;
  }

  // transform msg data into pointcloud data
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> image_data_cloud_origin(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud, *image_data_cloud_origin);
  if (image_data_cloud_origin->size() == 0) {
    ROS_ERROR("image_data_cloud_origin is empty.");
    return;
  }

  /* --------------------------- Filter out ---------------------------*/
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> image_data_cloud_downsize(new pcl::PointCloud<pcl::PointXYZ>);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> image_data_cloud_no_plane(new pcl::PointCloud<pcl::PointXYZ>);

  
  // downsize the cloud for faster and more efficient analysis with a filter
  voxel_grid.setLeafSize(0.001f, 0.001f, 0.95f); // Set leaf size 
  voxel_grid.setInputCloud(image_data_cloud_origin); 
  voxel_grid.filter(*image_data_cloud_downsize);
  if (image_data_cloud_downsize->size() == 0) {
    ROS_ERROR("image_data_cloud_downsize is empty. %ld", image_data_cloud_downsize->size());
    return;
  }
  // Initialize a vector to store the centroid of a cluster.
  Eigen::Vector4f centroid;

  // Create points in the camera and target frames
  // Initialize point structures to represent points in the camera and target frames.
  geometry_msgs::PointStamped point_in;
  geometry_msgs::PointStamped point_out;
  // Create a shared pointer to store the point cloud of the current cluster.
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> clusters(new pcl::PointCloud<pcl::PointXYZ>);
  calOrientation(image_data_cloud_origin, image_data_cloud_no_plane);
  if (image_data_cloud_no_plane->size() == 0) {
    ROS_ERROR("image_data_cloud_no_plane is empty. %ld", image_data_cloud_no_plane->size());
    return;
  }

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("/home/mhj/Desktop/image_data_cloud.pcd", *image_data_cloud_origin, false);
  writer.write<pcl::PointXYZ>("/home/mhj/Desktop/image_data_cloud_no_plane.pcd", *image_data_cloud_no_plane, false);

  writer.write<pcl::PointXYZ>("/home/mhj/Desktop/image_data_cloud_downsize.pcd", *image_data_cloud_downsize, false);
  
  

  // Create the filtering object
  // filter out ground points

  // Save the point cloud to a PCD file
  std::vector<pcl::PointIndices> cluster_indices_baskets_temp;
  ROS_INFO("====================== CALL BACK %ld, %ld", image_data_cloud_origin->size(), image_data_cloud_downsize->size());
  
  // 旋转矩阵->角度
  // Eigen::Vector3d eulerAngle=rotated_rotation.eulerAngles(0,1,2);
  // 输出世界坐标系下的四元数
  pcl::compute3DCentroid(*image_data_cloud_no_plane, centroid);
  // std::cout<<"===========centroid"<<centroid[0]<<", "<<centroid[1]<<", "<<centroid[2]<<std::endl;
  
  auto min_y_point = std::max_element(image_data_cloud_no_plane->begin(), image_data_cloud_no_plane->end(),
                                      [](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
                                          return p1.y   < p2.y;
                                      });
  auto max_y_point = std::max_element(image_data_cloud_no_plane->begin(), image_data_cloud_no_plane->end(),
                                      [](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
                                          return p1.y > p2.y;
                                      });
  auto min_x_point = std::max_element(image_data_cloud_no_plane->begin(), image_data_cloud_no_plane->end(),
                                      [](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
                                          return p1.x < p2.x;
                                      });
  auto max_x_point = std::max_element(image_data_cloud_no_plane->begin(), image_data_cloud_no_plane->end(),
                                      [](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
                                          return p1.x > p2.x;
                                      });
  min_y_p = min_y_point -> y;
  max_y_p = max_y_point -> y;
  min_x_p = min_x_point -> x;
  max_x_p = max_x_point -> x;


  

  
  // 输出各轴方向角度
  if(shape_type == "cross")
  {
    pcl::PassThrough<pcl::PointXYZ> pass;

    std::cout<<"!!!min_y_point="<<min_y_point->x<<min_y_point->y<<min_y_point->z<<"; max_y_point="<<max_y_point->x<<max_y_point->y<<max_y_point->z<<std::endl;
    
    std::cout<<"X:"<<angle<<std::endl;
    ROS_INFO("===CALCULATED ANGLE ALL: (%f)", (angle) * (180/pi_));

    centroid[1] += 1.5*length;

  }else{
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> image_data_cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
    angle_new += angle_offset_;
    // if (std::abs(min_y_p - max_y_p) < 5.01*length && std::abs(min_x_p - max_x_p) < 5.01*length) 
    // {
    //   if (angle_new > pi_/4){
    //     angle_new -= pi_/2;
    //   } else if (angle_new < -pi_/4){
    //     angle_new += pi_/2;
    //   }
    // }else{
    //   if (angle_new < pi_/4){
    //     angle_new -= pi_/2;
    //   } else if (angle_new > -pi_/4){
    //     angle_new += pi_/2;
    //   }
    // }
    double pi_ = 3.14159;
    centroid[0] += 2*length;

  }

  // Remove obtuse angles
  if (angle_new > pi_/2){
    angle_new -= pi_;
  } else if (angle_new < -pi_/2){
    angle_new += pi_;
  }
  // Minimise rotation to +- 45 degrees
  if (angle_new > pi_/4){
    angle_new -= pi_/2;
  } else if (angle_new < -pi_/4){
    angle_new += pi_/2;
  }
  angle -= angle_new;
  if (angle > pi_/2){
    angle -= pi_;
  } else if (angle < -pi_/2){
    angle += pi_;
  }
  ROS_INFO("===CALCULATED ANGLE all: (%f)", (angle) * (180/pi_));
  ROS_INFO("===CALCULATED ANGLE new: (%f)", (angle_new) * (180/pi_));
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_new);
  tf2::Quaternion q_result = q_x180deg * q_object;

  // 将 tf2::Quaternion 转换为 Eigen::Quaterniond
  Eigen::Quaterniond obj_quaternion(q_result.w(), q_result.x(), q_result.y(), q_result.z());

  std::cout<<"=====angle_offset_"<<angle<<std::endl;
  
  // 物体的旋转
  Eigen::Matrix3d quaternion_obj_rotation_matrix = obj_quaternion.normalized().toRotationMatrix();
  Eigen::Quaterniond e_quaternion_obj_rotation_matrix(quaternion_obj_rotation_matrix);
  
  geometry_msgs::PointStamped centroid_point;
  centroid_point.point.x = centroid[0];
  centroid_point.point.y = centroid[1];
  centroid_point.point.z = centroid[2];
  geometry_msgs::PointStamped centroid_point_rotated;
  geometry_msgs::TransformStamped msg_rotated_quaternion_camera;

  double eigen_w = e_quaternion_obj_rotation_matrix.w();
  double eigen_x = e_quaternion_obj_rotation_matrix.x();
  double eigen_y = e_quaternion_obj_rotation_matrix.y();
  double eigen_z = e_quaternion_obj_rotation_matrix.z();

  // 使用四元数的实部和虚部构造 tf2::Quaternion 对象
  // 将结果旋转矩阵转换为四元数
  tf2::Quaternion tf_quaternion_obj_rotation_matrix(eigen_x, eigen_y, eigen_z, eigen_w);
  tf2::convert(tf_quaternion_obj_rotation_matrix, msg_rotated_quaternion_camera.transform.rotation);
  tf2::doTransform(centroid_point.point, centroid_point_rotated.point, msg_rotated_quaternion_camera);

  // geometry_msgs/Quaternion rotation 四元数形式
  // 四元数：target_frame ~ camera_frame
  transform_matrix = tfTransform(target_frame, camera_frame);

  Eigen::Quaterniond quaternion(transform_matrix.transform.rotation.w,
                                   transform_matrix.transform.rotation.x,
                                   transform_matrix.transform.rotation.y,
                                   transform_matrix.transform.rotation.z);
  // -> wrold
  tf2::doTransform(centroid_point_rotated.point, point_out.point, transform_matrix);
  object_point = point_out;

  point_in.header.stamp = ros::Time::now();
  sub_flag_task1 = false;
  return;
}


/* Function Definition for Task 1 */
bool cw3::task1Move(geometry_msgs::PointStamped pick_point, geometry_msgs::PointStamped goal_point, 
  std::string shape_type)
{
  ROS_INFO("task1Move");
// 0.6; // Distance above object
  geometry_msgs::Pose view_pose;
  view_pose.position.z = 0.5;

  view_pose.position.x = pick_point.point.x - translation_hand_camera_x;
  view_pose.position.y = pick_point.point.y - translation_hand_camera_y;
  
  view_pose.orientation = orientation;
  
  /* Now perform the pick */
  bool success = true;
  ROS_INFO("Begining pick operation");
  // move the arm above the object
  success = moveArm(view_pose);
  if (!success) ROS_WARN("move not success");

  std::cout<<"---------------- pick before----"<<angle<<std::endl;
  transform_matrix = tfTransform(target_frame, camera_frame);
  std::cout<<"=======pp point_out:"<< transform_matrix.transform.rotation.x<<","<<transform_matrix.transform.rotation.y<<","<<transform_matrix.transform.rotation.z<<std::endl;
  angle = 0; 
  sub_flag_task1 = true;
  ros::Duration(0.1).sleep();
  while(sub_flag_task1)
  {
    ros::Duration(0.1).sleep();
  }
  std::cout<<"======= before orientation:"<< orientation.x<<","<<orientation.y<<","<<orientation.z<<std::endl;

  success = pickObj(getPose(pick_point));
  // success *= dropObj(getPose(goal_point));

  return success;
}



geometry_msgs::TransformStamped cw3::tfTransform(std::string t_frame, std::string s_frame)
{
  // Initialize a TF2 TransformListener object with the provided buffer
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tfListener(tf_buffer);
  geometry_msgs::TransformStamped transform_m;
  try 
  {
    // Wait for the transform to become available with a timeout of 1 second
    // This ensures that the transform is available before proceeding
    tf_buffer.canTransform(t_frame, s_frame, ros::Time(0), ros::Duration(1.0));

    // Get the latest transform between the target and camera frames
    transform_m = tf_buffer.lookupTransform(t_frame, s_frame, ros::Time(0));
        
  } catch(tf2::TransformException& ex) 
  {
    // Handle any exceptions that occur during the transform lookup
    ROS_WARN("Failed to transform point: %s", ex.what());
  }
  return transform_m;
}
geometry_msgs::PointStamped cw3::pointInWorld(float x, float y, float z, std::string s_frame, std::string t_frame)
{
  // Set the frame IDs for the points.
  point_in.header.frame_id = s_frame;
  point_out.header.frame_id = t_frame;
  // Set the coordinates of the centroid point in the camera frame.
  point_in.point.x = x;
  point_in.point.y = y;
  point_in.point.z = z;

  geometry_msgs::TransformStamped transform_m = tfTransform(t_frame, s_frame);
  // Transform the point to the target frame.
  tf2::doTransform(point_in.point, point_out.point, transform_m);
  
  return point_out;
}

void cw3::calOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
  std::cout << "---------------------- cal angle"<<angle<<std::endl;

  // segment: detect plane
  // Detecting ground surfaces using Random Sampling Consistency (RANSAC) plane segmentation
  pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.03);
  seg.setMaxIterations(1000);
  seg.setInputCloud(cloud);
  seg.segment (*plane_inliers, *plane_coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(plane_inliers);
  extract.setNegative(true);
  // Remove the planar inliers, extract the rest
  extract.filter(*cloud_filtered);
  if (cloud_filtered->size() == 0)
  {
    ROS_WARN("cloud_out has no data");
  }
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered);                    
  sor.setMeanK (50);                               
  sor.setStddevMulThresh (0.9);                    
  sor.filter (*cloud_out);                  
    
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("/home/mhj/Desktop/no_plane_image_cloud.pcd", *cloud_out, false);  
  writer.write<pcl::PointXYZ>("/home/mhj/Desktop/cloud_filtered.pcd", *cloud_filtered, false);  
  
  

  // 创建点云对象，用于存储拟合的直线点
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 创建点云对象，用于存储去除平面之后的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 创建分割对象
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // 设置分割参数
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.04);

  // 执行分割
  seg.setInputCloud(cloud_out);
  seg.segment(*inliers, *coefficients);

  // 提取拟合的直线上的点
  extract.setInputCloud(cloud_out);
  extract.setIndices(inliers);
  extract.filter(*line_cloud);

  // 将拟合的直线点云和原始点云合并
  *plane_removed_cloud = *cloud_out + *line_cloud;

  // 保存合并后的点云
  // 创建可视化对象
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  // 设置点云颜色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud(cloud_out, 255, 255, 255); // 白色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_line(line_cloud, 255, 0, 0); // 红色

  // 添加点云到可视化对象中，并设置颜色
  viewer->addPointCloud<pcl::PointXYZ>(cloud_out, color_cloud, "original_cloud");
  viewer->addPointCloud<pcl::PointXYZ>(line_cloud, color_line, "line_cloud");
  // 计算拟合直线的斜率
  float slope = coefficients->values[3] / coefficients->values[4];

  // 计算拟合直线的旋转角度（弧度）
  float angle_new = atan(slope);

  // 将弧度转换为角度
  float angle_deg = angle_new * 180.0 / M_PI;

  std::cout << "拟合直线的旋转角度（度）：" << angle_deg << std::endl;

  // 添加拟合直线到可视化对象中
  pcl::PointXYZ p1, p2;
  p1.x = coefficients->values[0] - 10.0;
  p1.y = coefficients->values[1] - 10.0 * slope;
  p1.z = coefficients->values[2] - 10.0;

  p2.x = coefficients->values[0] + 10.0;
  p2.y = coefficients->values[1] + 10.0 * slope;
  p2.z = coefficients->values[2] + 10.0;

  viewer->addLine(p1, p2, 0, 1, 0, "fitting_line");

  // 显示可视化窗口
  // while (!viewer->wasStopped()) {
  //     viewer->spinOnce();
  // }
  pcl::io::savePCDFile("/home/mhj/Desktop/merged_cloud.pcd", *plane_removed_cloud);
  

  ROS_INFO("===CALCULATED ANGLE: (%f)", (angle_new) * (180/pi_));

}

geometry_msgs::Quaternion cw3::getOrientation()
{
  std::cout<<"&&&&&&&&&& curent angle is: "<<angle<<std::endl;
  // define grasping as from abov
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_+angle);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);
  std::cout<<"===orientated position: "<<grasp_orientation.x<<","<<grasp_orientation.y<<","<<grasp_orientation.z<<std::endl;

  return grasp_orientation;
}

void cw3::tfCallback()
{
  // 相机和手之间的变换
  tf::StampedTransform transform;
  tf_listener.lookupTransform(hand_frame, camera_frame, ros::Time(), transform);
  tf::Vector3 translation_hand_camera = transform.getOrigin();
  translation_hand_camera_x = translation_hand_camera.getX();
  translation_hand_camera_y = translation_hand_camera.getY();
  translation_hand_camera_z = translation_hand_camera.getZ();

  std::cout<<"=== translation between hand and camera";
  ROS_INFO("Translation: (%f, %f, %f)", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  
}

void cw3::add_plane()
{
  geometry_msgs::Point plane_centre; // plane centre
  plane_centre.x = 0;
  plane_centre.y = 0;
  plane_centre.z = 0.01;                  // derive from cube center position and dimension
  geometry_msgs::Vector3 plane_dimension; // plane position
  plane_dimension.x = 5;
  plane_dimension.y = 5;
  plane_dimension.z = 0.01;
  geometry_msgs::Quaternion plane_orientation; // plane orientation
  plane_orientation.w = 1;
  plane_orientation.x = 0;
  plane_orientation.y = 0;
  plane_orientation.z = 0;
  addCollision("plane", plane_centre, plane_dimension, plane_orientation);
}

void cw3::addCollision(std::string object_name, geometry_msgs::Point centre, 
                             geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = target_frame;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}
/*
如何确保能抓住并不同朝向的cross? -> 如何获得物体的朝向，反馈给手臂角度
*/
