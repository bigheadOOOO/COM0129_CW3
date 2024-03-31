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
  sub_flag_task2 = false;
  Label = 0;
  sub_flag_task3 = false;
  sub_flag_task3_rgb = false;
  // sub_flag_task3_color = false;
  // define Orienation: 
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_+angle);
  tf2::Quaternion q_result = q_x180deg * q_object;
  orientation = tf2::toMsg(q_result);
  ROS_INFO("cw3 class initialised");
  add_plane();

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

  geometry_msgs::Pose view_pose;
  tfCallback();
  view_pose.position.z = 0.5;

  view_pose.position.x = 0.2;
  view_pose.position.y = 0;
  
  view_pose.orientation = orientation;
  
  /* Now perform the pick */
  bool success = true;
  ROS_INFO("Begining pick operation");
  // move the arm above the object
  success = moveArm(view_pose);
  if (!success) ROS_WARN("move not success"); 
  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  
  // Collect message in the service
  object_point = request.object_point;
  goal_point = request.goal_point;
  shape_type = request.shape_type;
  // add the plane as a collision
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
  
  // Extract message from the request
  ref_object_points = request.ref_object_points;
  mystery_object_point = request.mystery_object_point;

  task2Move(ref_object_points, mystery_object_point);  
  response.mystery_object_num = mysteryLabel;
  ROS_INFO("=============== Mystery = %d ===============", response.mystery_object_num);
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t3_callback(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  bool success = true;

  // Define the end-effector's pose
  geometry_msgs::Pose target_pose;
  // pre-defined end-effector orientation
  angle = 0;
  // target_pose.orientation = getOrientation();
  target_pose.orientation = orientation;

  // ================================= Find basket ================================= //

  target_pose.position.x = -0.45;
  target_pose.position.y = -0.3;
  target_pose.position.z = 0.856 / 1.5;
  bool isBasket = false;
  // intialize the basket position
  basket_object.x = 0;
  basket_object.y = 0;
  basket_object.z = 0;
  // the iteration time
  int iter = 0; 

  while (!isBasket && iter < 2) {
    ros::Duration(0.8).sleep(); // Sleep for 0.2 second
    success *= moveArm(target_pose);
    if (!success) ROS_WARN("move not success");

    // Get the image's point cloud
    image_data_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ros::Duration(0.8).sleep(); // Sleep for 0.2 second
    // Set the task 3's subscriber flag to be true and receive point cloud data
    sub_flag_task3 = true;
    ros::Duration(0.8).sleep(); // Sleep for 0.2 second
    // After receive the data, reset the task 3's subscriber flag to be false in order to save memory space
    sub_flag_task3 = false;

    isBasket = Task3_basketCheck();
    target_pose.position.y = 0.3;
    
    iter += 1;
  }

  std::cout << "====================Find basket y =" << basket_object.y << std::endl;

  // 
  ros::Duration(0.8).sleep();
  target_pose.position.x = 0.2;
  target_pose.position.y = 0;
  target_pose.position.z = 0.856 / 1.5;
  success *= moveArm(target_pose);
  ros::Duration(0.8).sleep();

  if (success){ROS_INFO("!!!!!!!!!!!!!!!!! Start point !!!!!!!!!!!!!");}
  // ================================= Find objects ================================= //
  /*
  Move to the position where all targets including boxes and baskets can be captured by the camera
     point 1: (0.62, 0.35, arm_height), orientation = 0
     point 2: (-0.1, -0.5, arm_height), orientation = pi/2
     point 3: (-0.63, 0, arm_height),   orientation = 0
     point 4: (-0.1, 0.5, arm_height),  orientation = -pi/2
  The searching sequence is 4->3->2->1

  1) If the basket is at left, adjust the search points to the right
  Case 1 point 3:
    target_pose.position.y = 0.35;
  Case 1 point 2:
    target_pose.position.x = 0.25;
  2) If the basket is at right, adjust the search points to the left
  Case 2 point 3:
    target_pose.position.y = -0.35;
  Case 2 point 4:
    target_pose.position.x = 0.25;  
  
  */ 

  // Set the basic search points

  search_position_1.x = 0.62;
  search_position_1.y = 0.35;
  search_position_1.z = arm_height;

  search_position_2.x = -0.1;
  search_position_2.y = -0.5;
  search_position_2.z = arm_height;

  search_position_3.x = -0.62;
  search_position_3.y = 0;
  search_position_3.z = arm_height;

  search_position_4.x = -0.1;
  search_position_4.y = 0.5;
  search_position_4.z = arm_height;

  // Adjust the search point based on the position of basket
  if (basket_object.y < 0) {
    search_position_2.x = search_position_2.x + basket_size;
    search_position_3.y = search_position_3.y + basket_size;
    
    search_position_4.x = search_position_2.x;

  }
  else {
    search_position_4.x = search_position_4.x + basket_size;
    search_position_3.y = search_position_3.y - basket_size;

    search_position_2.x = search_position_4.x;
  }

  // ============================== Search right area ============================== //
  ros::Duration(1.2).sleep();
  // move to the search pose
  target_pose.position = search_position_4;
  angle = -(3.14159 / 2.0);
  target_pose.orientation = getOrientation();
  success *= moveArm(target_pose);
  if (success){
    ROS_INFO("!!!!!!!!!!!!!!!!! point4 !!!!!!!!!!!!!");
  }
  ROS_INFO("!!!!!!!!!!!!!!!!! s4_x: %.3f !!!!!!!!!!!!!", search_position_4.x);
  ROS_INFO("!!!!!!!!!!!!!!!!! s4_y: %.3f !!!!!!!!!!!!!", search_position_4.y);

  if (!success){
    ROS_WARN("move not success");
  }
  ros::Duration(0.2).sleep();

  // Get the image's point cloud
  image_data_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  image_data_cloud_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  ros::Duration(1.2).sleep(); // Sleep for 1.2 second, waiting for the arm be stable

  // Set the task 3's subscriber flag to be true and receive point cloud data
  sub_flag_task3 = true;

  ros::Duration(0.8).sleep(); // Sleep for 0.8 second, waiting for the arm be stable
  // After receive the data, reset the task 3's subscriber flag to be false in order to save memory space
  sub_flag_task3 = false;

  // Obtain search waypoints
  Task3_waypointDetect();

  // ============================== Search back area ============================== //
  ros::Duration(1.2).sleep();
  // move to the search pose
  target_pose.position = search_position_3;
  angle = 0;
  target_pose.orientation = getOrientation();
  success *= moveArm(target_pose);
  if (success){
    ROS_INFO("!!!!!!!!!!!!!!!!! point3 !!!!!!!!!!!!!");
  }
  ROS_INFO("!!!!!!!!!!!!!!!!! s3_x: %.3f !!!!!!!!!!!!!", search_position_3.x);
  ROS_INFO("!!!!!!!!!!!!!!!!! s3_y: %.3f !!!!!!!!!!!!!", search_position_3.y);
  if (!success){ 
    ROS_WARN("move not success");
  }
  ros::Duration(0.2).sleep();

  // Get the image's point cloud
  image_data_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  image_data_cloud_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  ros::Duration(1.2).sleep(); // Sleep for 1.2 second, waiting for the arm be stable

  // Set the task 3's subscriber flag to be true and receive point cloud data
  sub_flag_task3 = true;
  ros::Duration(0.8).sleep(); // Sleep for 0.8 second, waiting for the arm be stable
  // After receive the data, reset the task 3's subscriber flag to be false in order to save memory space
  sub_flag_task3 = false;

  // Obtain search waypoints
  Task3_waypointDetect();

  // ============================== Search left area ============================== //
  ros::Duration(1.2).sleep();
  // move to the search pose
  target_pose.position = search_position_2;
  angle = (3.14159 / 2.0);
  target_pose.orientation = getOrientation();
  success *= moveArm(target_pose);
  if (success){ROS_INFO("!!!!!!!!!!!!!!!!! point2 !!!!!!!!!!!!!");}
  ROS_INFO("!!!!!!!!!!!!!!!!! s2_x: %.3f !!!!!!!!!!!!!", search_position_2.x);
  ROS_INFO("!!!!!!!!!!!!!!!!! s2_y: %.3f !!!!!!!!!!!!!", search_position_2.y);
  if (!success) ROS_WARN("move not success");
  ros::Duration(0.2).sleep();

  // Get the image's point cloud
  image_data_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  image_data_cloud_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  ros::Duration(1.2).sleep(); // Sleep for 1.2 second, waiting for the arm be stable

  // Set the task 3's subscriber flag to be true and receive point cloud data
  sub_flag_task3 = true;
  ros::Duration(0.8).sleep(); // Sleep for 0.8 second, waiting for the arm be stable
  // After receive the data, reset the task 3's subscriber flag to be false in order to save memory space
  sub_flag_task3 = false;
  
  // Obtain search waypoints
  Task3_waypointDetect();

  // ============================== Search front area ============================== //
  ros::Duration(1.2).sleep();
  // move to the search pose
  target_pose.position = search_position_1;
  angle = 0;
  target_pose.orientation = getOrientation();
  success *= moveArm(target_pose);
  if (success){
    ROS_INFO("!!!!!!!!!!!!!!!!! point1 !!!!!!!!!!!!!");
  }
  ROS_INFO("!!!!!!!!!!!!!!!!! s1_x: %.3f !!!!!!!!!!!!!", search_position_1.x);
  ROS_INFO("!!!!!!!!!!!!!!!!! s1_y: %.3f !!!!!!!!!!!!!", search_position_1.y);
  if (!success){
    ROS_WARN("move not success");
  }
  ros::Duration(0.2).sleep();

  // Get the image's point cloud
  image_data_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  image_data_cloud_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  ros::Duration(1.2).sleep(); // Sleep for 1.2 second, waiting for the arm be stable

  // Set the task 3's subscriber flag to be true and receive point cloud data
  sub_flag_task3 = true;
  ros::Duration(0.8).sleep(); // Sleep for 0.8 second, waiting for the arm be stable
  // After receive the data, reset the task 3's subscriber flag to be false in order to save memory space
  sub_flag_task3 = false;

  // Obtain search waypoints
  Task3_waypointDetect();



  // ============================== Search objects along waypoints ============================== //
  // Search objects in the waypoints list
  ros::Duration(1.2).sleep();
  success *= Task3_recognition();

  std::cout << "==================== shape object = " << shape_objects.size() << std::endl;
  std::cout << "==================== obstacle object = " << obstacle_objects.size() << std::endl;
  // total_num_shapes = obstacle_objects.size() + shape_objects.size();
  // response.total_num_shapes = total_num_shapes;

  int cross_num = std::count(shape_types.begin(), shape_types.end(), 2);
  int nought_num = std::count(shape_types.begin(), shape_types.end(), 1);

  response.total_num_shapes = cross_num + nought_num;

  if (cross_num >= nought_num) {
    most_common_shape = 2;
    response.num_most_common_shape = cross_num;
  }
  else {
    most_common_shape = 1;
    response.num_most_common_shape = nought_num;
  }

  ROS_INFO("!!!!!!!!!! shape = 2, Num: %d !!!!!!!!!!!", cross_num);
  ROS_INFO("!!!!!!!!!! shape = 1, Num: %d !!!!!!!!!!!", nought_num);
  ROS_INFO("!!!!!!! Most Common: %d, Num: %d !!!!!!!!", most_common_shape, response.num_most_common_shape);
  ROS_INFO("!!!!!!!!!! Total Num shapes: %d !!!!!!!!!", response.total_num_shapes);

  // ============================== Add obstacle as collision ============================== //
  if (obstacle_objects.size() != 0) {
    for (int i=0; i<obstacle_objects.size(); i++) {
      addCollision("obstacle_" + char(i), obstacle_objects[i], obstacle_dimensions[i], orientation);
    }
  }
  

  // ======================== Grasp one object belongs to the most common shape ======================== //
  goal_point.point = basket_object;
  if (most_common_shape == 1) {
    shape_type = "nought";
    auto shape_index = std::find(shape_types.begin(), shape_types.end(), 1) - shape_types.begin();
    object_point.point = shape_objects[shape_index];
  }
  else {
    shape_type = "cross";
    auto shape_index = std::find(shape_types.begin(), shape_types.end(), 2) - shape_types.begin();
    object_point.point = shape_objects[shape_index];
  }
  ros::Duration(0.8).sleep();
  task1Move(object_point, goal_point, shape_type);


  // Reset the recognistion list
  task3_waypoints = {};
  shape_objects = {};
  obstacle_objects = {};
  basket_object = {};

  shape_types = {};
  obstacle_dimensions = {};

  shape_area = {};
  obstacle_area = {};
  basket_area = {};

  return true;
}


//================================================================================================================================//
/****************************************************** Functions for Task 1 ******************************************************/

void cw3::Task1_imageCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{

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
  voxel_grid.setLeafSize(0.0001f, 0.0001f, 0.95f); // Set leaf size 
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
  calOrientation(image_data_cloud_downsize, image_data_cloud_no_plane);
  if (image_data_cloud_no_plane->size() == 0) {
    ROS_ERROR("image_data_cloud_no_plane is empty. %ld", image_data_cloud_no_plane->size());
    return;
  }

  pcl::PCDWriter writer;
  // writer.write<pcl::PointXYZ>("/home/mhj/Desktop/image_data_cloud.pcd", *image_data_cloud_origin, false);
  // writer.write<pcl::PointXYZ>("/home/mhj/Desktop/image_data_cloud_no_plane.pcd", *image_data_cloud_no_plane, false);

  // writer.write<pcl::PointXYZ>("/home/mhj/Desktop/image_data_cloud_downsize.pcd", *image_data_cloud_downsize, false);
  
  
  

  // Create the filtering object
  // filter out ground points

  // Save the point cloud to a PCD file
  std::vector<pcl::PointIndices> cluster_indices_baskets_temp;
  ROS_INFO("====================== CALL BACK %ld, %ld, %d", image_data_cloud_origin->size(), image_data_cloud_downsize->size(), image_data_cloud_no_plane->size());
  
  // 旋转矩阵->角度
  // Eigen::Vector3d eulerAngle=rotated_rotation.eulerAngles(0,1,2);
  // 输出世界坐标系下的四元数
  pcl::compute3DCentroid(*image_data_cloud_no_plane, centroid);


  std::cout<<"Angles:"<<angle<<", "<<angle_new<<std::endl;
  
  ROS_INFO("===CALCULATED ANGLE all: (%f)", (angle) * (180/pi_));
  ROS_INFO("===CALCULATED ANGLE new: (%f)", (angle_new) * (180/pi_));
  
  
  double pi_ =3.14159;

  // Remove obtuse angles
  if (angle_new > pi_/2){
    angle_new -= pi_;
  } else if (angle_new < -pi_/2){
    angle_new += pi_;
  }
 
  if (angle_new > pi_/4){
    angle_new -= pi_/2;
  } else if (angle_new < -pi_/4){
    angle_new += pi_/2;
  }


  angle += angle_new;
  // if (angle > pi_/2){
  //   angle -= pi_;
  // } else if (angle < -pi_/2){
  //   angle += pi_;
  // }
  // if (angle > pi_/4){
  //   angle -= pi_/2;
  // } else if (angle < -pi_/4){
  //   angle += pi_/2;
  // }
  // // 输出各轴方向角度
  // if(shape_type == "cross")
  // {
  //   // if (angle_new > )
  //   centroid[0] += 1.6*length;

  // }else{
  //   centroid[1] += 2*length;
  // }
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_new);
  tf2::Quaternion q_result = q_x180deg * q_object;

  // 将 tf2::Quaternion 转换为 Eigen::Quaterniond
  Eigen::Quaterniond obj_quaternion(q_result.w(), q_result.x(), q_result.y(), q_result.z());

  
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
  // 
  tf2::doTransform(centroid_point.point, centroid_point_rotated.point, msg_rotated_quaternion_camera);

  // // geometry_msgs/Quaternion rotation 四元数形式
  // // 四元数：target_frame ~ camera_frame
  // transform_matrix = tfTransform(target_frame, camera_frame);

  // Eigen::Quaterniond quaternion(transform_matrix.transform.rotation.w,
  //                                  transform_matrix.transform.rotation.x,
  //                                  transform_matrix.transform.rotation.y,
  //                                  transform_matrix.transform.rotation.z);
  // // -> wrold
  // tf2::doTransform(centroid_point_rotated.point, point_out.point, transform_matrix);
  // object_point = point_out;

  // point_in.header.stamp = ros::Time::now();
    // geometry_msgs/Quaternion rotation 四元数形式
  // 四元数：target_frame ~ camera_frame
  
  object_point.point = centroid_point_rotated.point;

  point_in.header.stamp = ros::Time::now();
  sub_flag_task1 = false;
  return;
}

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
  ros::Duration(0.8).sleep();
  // move the arm above the object
  success = moveArm(view_pose);
  if (!success) ROS_WARN("move not success");

  std::cout<<"---------------- pick before----"<<angle<<std::endl;
  transform_matrix = tfTransform(target_frame, camera_frame);
  angle = 0; 
  sub_flag_task1 = true;
  ros::Duration(0.4).sleep();
  //while(sub_flag_task1)
  //{
    ros::Duration(0.4).sleep();
  //}

  success = pickObj(getPose(pick_point));
  if (shape_type == "cross")  goal_point.point.x += 2*length;
  else{
    if (goal_point.point.y > 0) goal_point.point.y += 2*length;
    if (goal_point.point.y < 0) goal_point.point.y -= 2*length;

  }
  success *= dropObj(getPose(goal_point));
  //success *= moveArm(view_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
  }
  return success;
}



//================================================================================================================================//
/****************************************************** Functions for Task 2 ******************************************************/


void cw3::Task2_imageCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  /**
   * @brief The callback function for task 2 to obtain a point cloud "photo" without RGB
   * 
   */

  // Check if cloud_input_msg is valid
  if (cloud_input_msg->data.size() == 0) {
    ROS_ERROR("cloud_input_msg is empty.");
    return;
  }

  // Transform msg data into PCL data
  std::unique_ptr<pcl::PCLPointCloud2> cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*cloud_input_msg, *cloud);
  if (cloud->data.size() == 0) {
    ROS_ERROR("cloud is empty.");
    return;
  }

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> image_data_cloud_origin(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud, *image_data_cloud_origin);
  if (image_data_cloud_origin->size() == 0) {
    ROS_ERROR("image_data_cloud_origin is empty.");
    return;
  }

  /* --------------------------- Filter out ---------------------------*/
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> image_data_cloud_downsize(new pcl::PointCloud<pcl::PointXYZ>);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> image_data_cloud_no_plane(new pcl::PointCloud<pcl::PointXYZ>);

  
  // Downsize the cloud for faster and more efficient analysis with a filter
  voxel_grid.setLeafSize(0.0001f, 0.0001f, 0.95f); // Set leaf size 
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
  calOrientation(image_data_cloud_downsize, image_data_cloud_no_plane);
  if (image_data_cloud_no_plane->size() == 0) {
    ROS_ERROR("image_data_cloud_no_plane is empty. %ld", image_data_cloud_no_plane->size());
    return;
  }
  
  // Create the filtering object
  // filter out ground points

  std::vector<pcl::PointIndices> cluster_indices_baskets_temp;
  ROS_INFO("====================== CALL BACK %ld, %ld, %d", image_data_cloud_origin->size(), image_data_cloud_downsize->size(), image_data_cloud_no_plane->size());
  
  
  // Compute the 3D centroid in the camera frame
  pcl::compute3DCentroid(*image_data_cloud_no_plane, centroid);

  // Check the shape by counting the neighbor data around the centroid
  Task2_shapeRecognition(centroid, image_data_cloud_no_plane);
  
  
  // std::cout<<"Angles:"<<angle<<", "<<angle_new<<std::endl;
  
  // ROS_INFO("===CALCULATED ANGLE all: (%f)", (angle) * (180/pi_));
  // ROS_INFO("===CALCULATED ANGLE new: (%f)", (angle_new) * (180/pi_));
  
  
  // double pi_ =3.14159;

  // // Remove obtuse angles
  // if (angle_new > pi_/2){
  //   angle_new -= pi_;
  // } else if (angle_new < -pi_/2){
  //   angle_new += pi_;
  // }
 
  // if (angle_new > pi_/4){
  //   angle_new -= pi_/2;
  // } else if (angle_new < -pi_/4){
  //   angle_new += pi_/2;
  // }


  // angle += angle_new;
  
  // tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // // determine the grasping orientation
  // tf2::Quaternion q_object;
  // q_object.setRPY(0, 0, angle_new);
  // tf2::Quaternion q_result = q_x180deg * q_object;

  // // 将 tf2::Quaternion 转换为 Eigen::Quaterniond
  // Eigen::Quaterniond obj_quaternion(q_result.w(), q_result.x(), q_result.y(), q_result.z());

  
  // // 物体的旋转
  // Eigen::Matrix3d quaternion_obj_rotation_matrix = obj_quaternion.normalized().toRotationMatrix();
  // Eigen::Quaterniond e_quaternion_obj_rotation_matrix(quaternion_obj_rotation_matrix);
  
  // geometry_msgs::PointStamped centroid_point;
  // centroid_point.point.x = centroid[0];
  // centroid_point.point.y = centroid[1];
  // centroid_point.point.z = centroid[2];
  // geometry_msgs::PointStamped centroid_point_rotated;
  // geometry_msgs::TransformStamped msg_rotated_quaternion_camera;

  // double eigen_w = e_quaternion_obj_rotation_matrix.w();
  // double eigen_x = e_quaternion_obj_rotation_matrix.x();
  // double eigen_y = e_quaternion_obj_rotation_matrix.y();
  // double eigen_z = e_quaternion_obj_rotation_matrix.z();
  
  // // 使用四元数的实部和虚部构造 tf2::Quaternion 对象
  // // 将结果旋转矩阵转换为四元数
  // tf2::Quaternion tf_quaternion_obj_rotation_matrix(eigen_x, eigen_y, eigen_z, eigen_w);
  // tf2::convert(tf_quaternion_obj_rotation_matrix, msg_rotated_quaternion_camera.transform.rotation);
  // // 
  // tf2::doTransform(centroid_point.point, centroid_point_rotated.point, msg_rotated_quaternion_camera);
  
  
  // objectGraspPosition.push_back(centroid_point_rotated.point.x);
  // objectGraspPosition.push_back(centroid_point_rotated.point.y);
  // objectGraspPosition.push_back(centroid_point_rotated.point.z);

  // point_in.header.stamp = ros::Time::now();
  // Reset the flag for task 2's image callback as false
  sub_flag_task2 = false;
  
  return;
}


void cw3::Task2_shapeRecognition(const Eigen::Vector4f centroid, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> image_data_cloud_no_plane)
{
  /**
   * @brief recognize the object's shape by checking whether there is any point around the centroid, using KDTree
   * 
   */

  // Define the position of interest - centroid
    pcl::PointXYZ search_point;
    search_point.x = centroid[0];
    search_point.y = centroid[1];
    search_point.z = centroid[2];

    // Create a KD tree for efficient search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(image_data_cloud_no_plane);

    // Define search parameters
    // Define a search radius around the point of interest
    double search_radius = 0.01; 

    // Search for points within the search radius
    std::vector<int> point_indices;
    std::vector<float> point_distances;

    ROS_INFO("======================= Object Shape Recognition ======================");

    // Ensure the object is not obstacle
    if (kdtree.radiusSearch(search_point, search_radius, point_indices, point_distances) > 0 ) {
        ROS_INFO("!!!!!!!!!!!!!!!!! CROSS !!!!!!!!!!!!!!!!!!!");
        Label = 2; // the cross label is 2
        return;           
    } else {
        // No points found within the search radius
        Label = 1; // the nought label is 1
        ROS_INFO("!!!!!!!!!!!!!!!!! DONOUGHT !!!!!!!!!!!!!!!!!!!");
    }

    return;

}


float cw3::task2Move(std::vector<geometry_msgs::PointStamped> ref_object_points, geometry_msgs::PointStamped mystery_object_point)
{
  ROS_INFO("task2Move");
  geometry_msgs::Pose view_pose;
  view_pose.position.z = 0.5;

  int len = ref_object_points.size();
  ROS_INFO("Number of reference objects: %d", len);


  for(int i = 0; i< ref_object_points.size(); i++){
    view_pose.position.x = ref_object_points[i].point.x - translation_hand_camera_x;
    view_pose.position.y = ref_object_points[i].point.y - translation_hand_camera_y;
  
    view_pose.orientation = orientation;

    /* Now Take A Picture for The Reference Object*/
    bool success = true;
    ROS_INFO("Begining photographing operation");
    ros::Duration(0.8).sleep(); // Sleep for 0.8 second
    // move the arm above the object
    success = moveArm(view_pose);
    if (!success) ROS_WARN("move not success");

  
    std::cout<<"---------------- pick before----"<<angle<<std::endl;
    transform_matrix = tfTransform(target_frame, camera_frame);
    angle = 0;
    sub_flag_task2 = true;
    Label = 0; // reset the label
    ros::Duration(0.2).sleep();
    //while(sub_flag_task2){
    ros::Duration(0.2).sleep();
    //}
    shapeLabel[Label]++;
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  label: %d, num: %d !!!!!!!!!!!!!!!!!!!!!!!!", Label, shapeLabel[Label]);
  }

  // For the mystery point
  view_pose.position.x = mystery_object_point.point.x - translation_hand_camera_x;
  view_pose.position.y = mystery_object_point.point.y - translation_hand_camera_y;
  
  view_pose.orientation = orientation;

  /* Now Take A Picture for The Reference Object*/
  bool success = true;
  ROS_INFO("Begin photographing operation");
  // move the arm above the object
  success = moveArm(view_pose);
  if (!success) ROS_WARN("move not success");

  
  std::cout<<"---------------- pick before----"<<angle<<std::endl;
  transform_matrix = tfTransform(target_frame, camera_frame);
  angle = 0;
  sub_flag_task2 = true;
  Label = 0;  // reset the label
  ros::Duration(0.1).sleep();
  while(sub_flag_task2)
  {
    ros::Duration(0.1).sleep();
  }
  shapeLabel[Label]++;
  mysteryLabel = Label;
  ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Mystery Object's Label: %d, num: %d !!!!!!!!!!!!!!!!!!!!!!!!", Label, shapeLabel[Label]);
  
  

  return 1.0;


}


//================================================================================================================================//
/****************************************************** Functions for Task 3 ******************************************************/

void cw3::extractObjectsClustersRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud, std::vector<pcl::PointIndices>& cluster_indices, 
                                float tolerance, int min_cluster_size, int max_cluster_size)
{
  /**
   * @brief Extract baskets clusters from the RGB point cloud and store the cluster indices in cluster_indices
   * 
   */
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setInputCloud(color_cloud);
  ec.setClusterTolerance(tolerance);          // Set the tolerance for cluster segmentation
  ec.setMinClusterSize(min_cluster_size);     // Set the minimum number of points in a cluster
  ec.setMaxClusterSize(max_cluster_size);     // Set the maximum number of points in a cluster
  ec.extract(cluster_indices);                // Extracting and storing the indices of the clustered point cloud
}

void cw3::Task3_imageCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{ 
  /**
   * @brief The callback function for task 3 to obtain a point cloud "photo" without RGB
   * 
   */

  // Check if cloud_input_msg is valid
  if (cloud_input_msg->data.size() == 0) {
    ROS_ERROR("cloud_input_msg is empty.");
    return;
  }
  
  // Convert the msg data to PCL data
  std::unique_ptr<pcl::PCLPointCloud2> cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*cloud_input_msg, *cloud);
  if (cloud->data.size() == 0) {
    ROS_ERROR("cloud is empty.");
    return;
  }

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> image_data_cloud_origin(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud, *image_data_cloud_origin);
  if (image_data_cloud_origin->size() == 0) {
    ROS_ERROR("image_data_cloud_origin is empty.");
    return;
  }

  /* --------------------------- Filter out ---------------------------*/
  // Downsize the cloud for faster and more efficient analysis with a filter
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> image_data_cloud_downsize(new pcl::PointCloud<pcl::PointXYZ>);

  voxel_grid.setLeafSize(0.0001f, 0.0001f, 0.95f); // Set leaf size 
  voxel_grid.setInputCloud(image_data_cloud_origin); 
  voxel_grid.filter(*image_data_cloud_downsize);
  if (image_data_cloud_downsize->size() == 0) {
    ROS_ERROR("image_data_cloud_downsize is empty. %ld", image_data_cloud_downsize->size());
    return;
  }

  
  // Detect ground surfaces using Random Sampling Consistency (RANSAC) plane segmentation
  // segment: detect plane
  pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ());

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.0325);
  seg.setMaxIterations(1000);
  seg.setInputCloud(image_data_cloud_downsize);
  seg.segment (*plane_inliers, *plane_coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(image_data_cloud_downsize);
  extract.setIndices(plane_inliers);
  extract.setNegative(true);

  // Remove the planar inliers, extract the rest
  extract.filter (*image_data_cloud);
  if (image_data_cloud->size() == 0) {
    ROS_WARN("cloud_out has no data");
  }
                

  std::cout << "PointCloud representing the planar component: " << image_data_cloud->size () << " data points." << std::endl;
  // Reset the flag for task 3's image callback as false
  sub_flag_task3 = false;

  return;
}


void cw3::Task3_imageCallback_color(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  /**
   * @brief The callback function for task 3 to obtain a point cloud "photo" with RGB
   * 
   */
  
  // Check if cloud_input_msg is valid
  if (cloud_input_msg->data.size() == 0) {
    ROS_ERROR("cloud_input_msg is empty.");
    return;
  }
  
  // Transform msg data into PCL data
  std::unique_ptr<pcl::PCLPointCloud2> cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*cloud_input_msg, *cloud);
  if (cloud->data.size() == 0) {
    ROS_ERROR("cloud is empty.");
    return;
  }

  
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> image_data_cloud_origin(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*cloud, *image_data_cloud_origin);
  if (image_data_cloud_origin->size() == 0) {
    ROS_ERROR("image_data_cloud_origin is empty.");
    return;
  }

  /* --------------------------- Filter out ---------------------------*/
  // Downsize the cloud for faster and more efficient analysis with a filter
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> image_data_cloud_downsize(new pcl::PointCloud<pcl::PointXYZRGB>);

  voxel_grid_color.setLeafSize(0.0001f, 0.0001f, 0.95f); // Set leaf size 
  voxel_grid_color.setInputCloud(image_data_cloud_origin); 
  voxel_grid_color.filter(*image_data_cloud_downsize);
  if (image_data_cloud_downsize->size() == 0) {
    ROS_ERROR("image_data_cloud_downsize is empty. %ld", image_data_cloud_downsize->size());
    return;
  }

  // Detect ground surfaces using Random Sampling Consistency (RANSAC) plane segmentation
  // segment: detect plane
  pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ());

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.0325);
  seg.setMaxIterations(1000);
  seg.setInputCloud(image_data_cloud_downsize);
  seg.segment (*plane_inliers, *plane_coefficients);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(image_data_cloud_downsize);
  extract.setIndices(plane_inliers);
  extract.setNegative(true);

  // Remove the planar inliers, extract the rest
  extract.filter (*image_data_cloud_rgb);

  if (image_data_cloud_rgb->size() == 0) {
    ROS_WARN("cloud_out has no data");
  }


  std::cout << "PointCloud representing the planar component: " << image_data_cloud_rgb->size () << " data points." << std::endl;
  // Reset the flag for task 3's image callback_color as false
  sub_flag_task3_rgb = false;


  return;
}


void cw3::Task3_shapeRecognition(const Eigen::Vector4f centroid, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> image_data_cloud_no_plane)
{
  /**
   * @brief recognize the object's shape by checking whether there is any point around the centroid, using KDTree
   * 
   */

  // Define the position of interest - centroid
    pcl::PointXYZRGB search_point;
    search_point.x = centroid[0];
    search_point.y = centroid[1];
    search_point.z = centroid[2];

    // Create a KD tree for efficient search
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(image_data_cloud_no_plane);

    // Define search parameters
    // Define a search radius around the point of interest
    double search_radius = 0.01; 

    // Search for points within the search radius
    std::vector<int> point_indices;
    std::vector<float> point_distances;

    ROS_INFO("======================= Object Shape Recognition ======================");

    // Ensure the object is not obstacle
    obstacleColor = colorFilter(image_data_cloud_no_plane, centroid);

    if (kdtree.radiusSearch(search_point, search_radius, point_indices, point_distances) > 0 ) {
        
        if(obstacleColor != 2 && obstacleColor != 3){
          ROS_INFO("!!!!!!!!!!!!!!! CROSS !!!!!!!!!!!!!!!!!");
          Label = 2; // the cross label is 2
          return;
        }
                
    } else {
        // No points found within the search radius
        Label = 1; // the nought label is 1
        ROS_INFO("!!!!!!!!!!!!!!! NOUGHT !!!!!!!!!!!!!!!!!!");
    }

    return;

}


bool cw3::Task3_basketCheck()
{ 
  /**
   * @brief Check the basket's exsitence and position
   * 
   */

  bool existBasket = false;

  std::vector<pcl::PointIndices> cluster_indices;
  extractObjectsClusters(image_data_cloud, cluster_indices, 0.01, 200, 50000);

  
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*image_data_cloud)[idx]);
    }
    
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;



    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

    float lenth = abs(maxPt.x - minPt.x);

    // Check the size of the cluster
    if (lenth > 0.3) {
      // Compute the centroid of the basket in world frame
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud_cluster, centroid);

      geometry_msgs::PointStamped centroid_world_point;
      centroid_world_point = pointInWorld(centroid[0], centroid[1], centroid[2], camera_frame, target_frame);

      std::cout << "==================== Basket point: " << centroid_world_point.point << std::endl;

      basket_object = centroid_world_point.point;
      existBasket = true;
      break;
    }
  }

  return existBasket;
}


void cw3::Task3_waypointDetect()
{ 
  /**
   * @brief Append each object's position to the waypoint list for accurate checking later
   * 
   */
  ROS_INFO("======= Waypoint Detection start =======");

  std::vector<pcl::PointIndices> cluster_indices;
  // Divide the point clouds into clusters ( = objects)
  extractObjectsClusters(image_data_cloud, cluster_indices, 0.01, 200, 50000);

  // The centroid of each object, in the camera frame
  Eigen::Vector4f centroid;

  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*image_data_cloud)[idx]);
    }
    
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Compute the 3D centroid of each object in the camera frame
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    
    // Transform the 3D centroid from camera frame to world frame
    geometry_msgs::PointStamped centroid_world_point;
    centroid_world_point = pointInWorld(centroid[0], centroid[1], centroid[2], camera_frame, target_frame);

    // Define the pose of each waypoint
    geometry_msgs::Pose waypoint_centroid;

    waypoint_centroid.position = centroid_world_point.point;
    waypoint_centroid.position.z = waypoint_centroid.position.z * 1.5 + 0.45;
    //waypoint_centroid.position.z = waypoint_centroid.position.z * 1.5 + 0.35;
    waypoint_centroid.orientation = orientation; 

    // Append each object's centroid pose to the waypoint list for accurate checking later
    task3_waypoints.push_back(waypoint_centroid);
  }
  
  return;
}


bool cw3::Task3_recognition()
{ 
  /**
   * @brief recognize the shape (obstacle or object, object's type) at each waypoint
   * 
   */

  if (task3_waypoints.size() == 0) {
    ROS_ERROR("No object detected.");
    return true;
  }
  ROS_INFO("======== Waypoint Recognition Start =========");
  std::cout << "=============== Waypoints Num: " << task3_waypoints.size() << std::endl;

  bool success;
  

  geometry_msgs::Pose target_pose;
  for (int wpi = 0; wpi < task3_waypoints.size(); wpi++) {

    bool non_cloud = false;
    target_pose = task3_waypoints[wpi];

    int iter = 0;
    Eigen::Vector4f eigen_centroid_interest;
    float area_range;
    float max_height;

    // =================== Extract the current interest object and move above the object ====================== //
    while (iter < 14) {
      // Move to the target point
      ros::Duration(1.2).sleep();     // Sleep for 1.2 second, waiting the arm to be stable
      success = false;
      success = moveArm(target_pose);
      if (!success) {
        ROS_WARN("move not success");
        
      }
      
      // ========= Get current filted point cloud ======== //

      image_data_cloud_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
      ros::Duration(0.8).sleep(); // Sleep for 0.8 second, waiting the arm to be stable
      // Set the task 3's subscriber flag (RGB) to be true and receive point cloud data
      sub_flag_task3_rgb = true;
      ros::Duration(0.8).sleep(); // Sleep for 0.8 second, waiting the arm to be stable
      // After receive the data, reset the task 3's subscriber flag to be false in order to save memory space
      sub_flag_task3_rgb = false;

      if (image_data_cloud_rgb -> size() == 0) {
        ROS_WARN("cloud has no data");
        non_cloud = true;
        break;
      }

      // ========== Identify which cluster cloud is our current interest cloud ========= //
      std::vector<pcl::PointIndices> cluster_indices;

      // Divide the RGB point cloud into clusters
      extractObjectsClustersRGB(image_data_cloud_rgb, cluster_indices, 0.01, 2000, 100000);

      std::vector<geometry_msgs::PointStamped> centroid_list = {};
      Eigen::Vector4f centroid;
      std::vector<Eigen::Vector4f> eigen_centroid_list = {};
      std::vector<float> area_list = {};
      std::vector<geometry_msgs::PointStamped> max_z_point_list = {};
      
      geometry_msgs::PointStamped centroid_world_point;

      // Append the area of the detected object into the area_list
      for (const auto& cluster : cluster_indices)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        for (const auto& idx : cluster.indices) {
          cloud_cluster -> push_back((*image_data_cloud_rgb)[idx]);

        }
       
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

        // Compute centroid
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        eigen_centroid_list.push_back(centroid);
        centroid_world_point = pointInWorld(centroid[0], centroid[1], centroid[2], camera_frame, target_frame);
        centroid_list.push_back(centroid_world_point);

        // Compute the max height of the object
        max_z_point_list.push_back(pointInWorld(centroid[0], centroid[1], maxPt.z, camera_frame, target_frame));

        // Compute the largest x and y coordinates
        float x_range = maxPt.x - minPt.x;
        float y_range = maxPt.y - minPt.y;
        float xy_area;
        if (x_range >= y_range) xy_area = x_range;
        else xy_area = y_range;
        area_list.push_back(xy_area);
      }
      
      // ========== Compute the distances between the observed objects and camera =========== //
      float dist;
      std::vector<float> dist_list = {};
      for (int cloud_i=0; cloud_i<centroid_list.size(); cloud_i++) {
        dist = calDistance_xy(centroid_list[cloud_i].point, target_pose.position);
        dist_list.push_back(dist);
      }
      int min_dist = std::min_element(dist_list.begin(),dist_list.end()) - dist_list.begin();

      // ========= Check if current pose is above the centroid of the object =========== //
      if (dist_list[min_dist] < threshold) {
        // Record the importance information of interest object
        eigen_centroid_interest = eigen_centroid_list[min_dist];
        area_range = area_list[min_dist];
        max_height = max_z_point_list[min_dist].point.z;
        break;
      }
      target_pose.position.x = centroid_list[min_dist].point.x;
      target_pose.position.y = centroid_list[min_dist].point.y;

      iter += 1;

      if (iter == 12) {
        ROS_WARN("Failed to go to object position.");
        // Record the importance information of interest object
        eigen_centroid_interest = eigen_centroid_list[min_dist];
        area_range = area_list[min_dist];
        max_height = max_z_point_list[min_dist].point.z;
      }
    }

    if (non_cloud == true) continue;

    // ======================== Recognize object type and save it to the list ============================ //
    obstacleColor = colorFilter(image_data_cloud_rgb, eigen_centroid_interest);
    if (!isDetected(target_pose.position)) {
      std::cout << "=================== New object" << std::endl;
      // Obstacle, 0.055
      if (obstacleColor == 2) {
        std::cout << "=================== height = " << max_height << ", Obstacle" << std::endl;
        obstacle_objects.push_back(target_pose.position);

        obstacle_area.push_back(area_range);
        geometry_msgs::Vector3 dimensions;
        dimensions.x = area_range;
        dimensions.y = area_range;
        dimensions.z = max_height;
        obstacle_dimensions.push_back(dimensions);
      }
      // Shape object
      if (obstacleColor == 1) {
        std::cout << "=================== height = " << max_height << ", Shape" << std::endl;
        eigen_centroid_interest[2] =  0.0;
        for (auto& point : image_data_cloud_rgb->points) {
          point.z = 0.0;
        }

        Task3_shapeRecognition(eigen_centroid_interest, image_data_cloud_rgb);
        shape_objects.push_back(target_pose.position);
        shape_objects.push_back(target_pose.pose);
        shape_types.push_back(Label);
        shape_area.push_back(area_range);
        Label = 0;
      }

    }
  }

  std::cout << "============== Finish searching ==========" << std::endl;
  return true;
}


float cw3::calDistance_xy(geometry_msgs::Point a, geometry_msgs::Point b)
{
  /**
   * @brief Calculate the distance between two points
   */

  float dist;
  dist = sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y, 2));
  return dist;
}


bool cw3::isDetected(geometry_msgs::Point current_object)
{
  /**
   * @brief Check the currently observed object is detected before or not
   * 
   */

  float dist;

  // Search shape object
  for (int i = 0; i<shape_objects.size(); i++) {
    dist = calDistance_xy(current_object, shape_objects[i]);
    if ( dist <= (shape_area[i]/2) ) {
      std::cout << "=================== Exist shape" << std::endl;
      return true;
    }
  }

  // Search obstacle object
  for (int i = 0; i<obstacle_objects.size(); i++) {
    dist = calDistance_xy(current_object, obstacle_objects[i]);
    if ( dist <= (obstacle_area[i]/2) ) {
      std::cout << "=================== Exist obstacle" << std::endl;
      return true;
    }
  }

  // Search basket
  dist = calDistance_xy(current_object, basket_object);
  if ( dist <= (basket_area/2) ) {
    std::cout << "=================== Exist basket" << std::endl;
    return true;
  }

  return false;
}


int cw3::colorFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const Eigen::Vector4f centroid) 
{
  /**
   * @brief Check the RGB value at the centroid to determine the object's type: ideal object, obstacle, background
   *
   */
    
    ROS_INFO("=================== Color Detect ================!");
    // Find the index of the point closest to the centroid
    float min_distance = std::numeric_limits<float>::max();
    int centroid_index = -1;
    for (size_t i = 0; i < cloud_in->size(); ++i) {
        pcl::PointXYZRGB point = cloud_in->points[i];
        Eigen::Vector3f point_position(point.x, point.y, point.z);
        float distance = (point_position - centroid.head<3>()).norm();
        if (distance < min_distance) {
            min_distance = distance;
            centroid_index = i;
        }
    }

    // Access RGB value at the centroid
    pcl::PointXYZRGB centroid_point = cloud_in -> points[centroid_index];

    ROS_INFO("!!!!! CENTROID R: %d, G: %d, B: %d !!!!!!!!!", centroid_point.r, centroid_point.g, centroid_point.b);

    // Obstacle
    if (centroid_point.r == centroid_point.b && centroid_point.r == centroid_point.g && centroid_point.r < 35) {
      return 2;
    }
    // Background
    else if(centroid_point.r == centroid_point.b && centroid_point.r == centroid_point.g && centroid_point.r > 35){
      return 3;
    }
    // Ideal object
    else{
      return 1;
    }
    
}


geometry_msgs::Quaternion cw3::getOrientation()
{
  /**
   * @brief Determine the orientation of the end-effector based on the global attribute "angle"
   *
   */

  std::cout<<"&&&&&&&&&& curent angle is: "<<angle<<std::endl;
  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_+angle);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);
  std::cout<<"=== orientated position: "<<grasp_orientation.x<<","<<grasp_orientation.y<<","<<grasp_orientation.z<<std::endl;

  return grasp_orientation;
}




//================================================================================================================================//
/************************************************* Basic Functions for All Tasks **************************************************/

void cw3::imageCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Only start to store the pointcloud message after task begins
  if (!sub_flag_task1 && !sub_flag_task2 && !sub_flag_task3 && !sub_flag_task3_rgb) return; 
  
  // Task 1
  if (sub_flag_task1) {
    Task1_imageCallback(cloud_input_msg);
    return;
  }
  
  // Task 2
  if (sub_flag_task2) {
    Task2_imageCallback(cloud_input_msg);
    return;
  }

  // Task 3 - obtain the point cloud without RGB
  if (sub_flag_task3) {
    Task3_imageCallback(cloud_input_msg);
    return;
  }
  
  // Task 3 - obtain the point cloud with RGB
  if (sub_flag_task3_rgb) {
    Task3_imageCallback_color(cloud_input_msg);
    return;
  }
}

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



bool 
cw3::moveArm(geometry_msgs::Pose target_pose)
{
  /**
  * @brief Move the robot arm to a target pose.
  * 
  * @param target_pose The target pose to move the arm to.
  * @return True if the movement was successful, false otherwise.
  */

  // Step 1: Setup the target pose
  ROS_INFO("moveArm: Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // // Step 2: Create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");

  // // 设置笛卡尔路径规划参数
  // std::vector<geometry_msgs::Pose> waypoints;
  // waypoints.push_back(target_pose); // 添加抓取目标姿态
  // // 添加更多的路径点，用于描述笛卡尔路径

  // 执行笛卡尔路径规划
  double eef_step = 0.01; // 设置步长
  double jump_threshold = 0.0; // 设置跳跃阈值
  moveit_msgs::RobotTrajectory trajectory;
  bool avoid_collisions = true;
  moveit_msgs::MoveItErrorCodes error_code;

  // 进行笛卡尔路径规划
  bool success = arm_group_.computeCartesianPath({target_pose}, eef_step, jump_threshold, trajectory, avoid_collisions);

  // bool success = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions, &error_code);

  // 检查规划结果
  if (success) {
      ROS_INFO("Cartesian path planning succeeded");
      
      // 创建运动计划
      moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
      cartesian_plan.trajectory_ = trajectory;

      // 可视化规划结果
      ROS_INFO("Visualizing Cartesian path plan");
      arm_group_.execute(cartesian_plan);
  } else {
    ROS_ERROR("Cartesian path planning failed with error code %d", error_code.val);
    // setup the target pose
    ROS_INFO("Setting pose target");
    arm_group_.setPoseTarget(target_pose);

    // create a movement plan for the arm
    ROS_INFO("Attempting to plan the path");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_group_.plan(my_plan) ==
      moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

    // execute the planned path
    arm_group_.move();
  }

  return success;

  // Step 1: Setup the target pose
  // ROS_INFO("moveArm: Setting pose target");
  // arm_group_.setPoseTarget(target_pose);

  // // Step 2: Create a movement plan for the arm
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // bool success = (arm_group_.plan(my_plan) ==
  //   moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // // Step 3: Log the success of planning the path
  // ROS_INFO("Visualising plan %s", success ? "" : "FAILED");
  // if (not success) return success;
  // // Step 4: Execute the planned path
  // arm_group_.move();

  // // Return the success status of the movement
  // return true;
}


bool 
cw3::moveGripper(float width)
{
  /**
  * @brief Move the robot gripper to a specified width.
  * 
  * @param width The desired width of the gripper.
  * @return True if the movement was successful, false otherwise.
  */

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

bool 
cw3::pickObj(geometry_msgs::Pose pose)
{
  /**
   * @brief Pick up the object with an input pose
   * 
   */

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
  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
  }
  transform_matrix = tfTransform(target_frame, camera_frame);

  Eigen::Quaterniond quaternion(transform_matrix.transform.rotation.w,
                                   transform_matrix.transform.rotation.x,
                                   transform_matrix.transform.rotation.y,
                                   transform_matrix.transform.rotation.z);
  // -> wrold
  
  tf2::doTransform(object_point, object_point, transform_matrix);
  if (shape_type == "cross") 
  {
    approach_pose.position.x = object_point.point.x + 1.4*length;
    approach_pose.position.y = object_point.point.y;
  }
  else{
    approach_pose.position.x = object_point.point.x;
    approach_pose.position.y = object_point.point.y + 2*length;
  }
  grasp_pose.position.x = approach_pose.position.x;
  grasp_pose.position.y = approach_pose.position.y;
  
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
  /**
   * @brief Drop the object with an input pose
   * 
   */

  // move arm: go to the basket
  geometry_msgs::Pose drop_pose;
  drop_pose = pose;
  drop_pose.position.z += approach_distance_basket;
  drop_pose.position.z += z_offset_obj+3*length;
  // drop middile pose
  drop_pose.position.x *= -1;
  // move arm to drop pose: plus cube_length to avoid the box collide with the basket
  int success = moveArm(drop_pose); 
  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed. move arm to drop pose");
  }
  // approach pose
  drop_pose.position.x *= -1;
  drop_pose.position.z -= length;
  success = moveArm(drop_pose); 
  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed. move arm to drop pose");
    // move arm to drop pose
    success = moveArm(drop_pose); 
  }

  // drop pose
  drop_pose.position.z -= 3*length;
  success = moveArm(drop_pose); 
  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed. move arm to drop pose");
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
  drop_pose.position.z += basket_h+approach_distance_basket+2.5*length;
  drop_pose.position.x *= -1;
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
  seg.setDistanceThreshold (0.0325);
  seg.setMaxIterations(1000);
  seg.setInputCloud(cloud);
  seg.segment (*plane_inliers, *plane_coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(plane_inliers);
  extract.setNegative(true);
  // Remove the planar inliers, extract the rest
  extract.filter(*cloud_out);
  if (cloud_out->size() == 0)
  {
    ROS_WARN("cloud_out has no data");
  }
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud (cloud_filtered);                    
  // sor.setMeanK (50);                               
  // sor.setStddevMulThresh (0.9);                    
  // sor.filter (*cloud_out);                  
  for (auto& point : cloud_out->points) {
    point.z = 0.0;
  }
  pcl::PCDWriter writer;
  // writer.write<pcl::PointXYZ>("/home/mhj/Desktop/no_plane_image_cloud.pcd", *cloud_out, false);  
  // writer.write<pcl::PointXYZ>("/home/mhj/Desktop/cloud_filtered.pcd", *cloud_filtered, false);  
  
  

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
  seg.setDistanceThreshold(0.02);
  // seg.setAxis(Eigen::Vector3f(0, 0, 1)); // 设置忽略 Z 轴数据
  // 执行分割
  seg.setInputCloud(cloud_out);
  seg.segment(*inliers, *coefficients);
  std::cout<<inliers->indices.size()<<coefficients->values.size();
  // 提取拟合的直线上的点
  extract.setInputCloud(cloud_out);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*line_cloud);

  // 将拟合的直线点云和原始点云合并
  *plane_removed_cloud = *cloud_out + *line_cloud;

  // 保存合并后的点云
  // 创建可视化对象
  // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  // viewer->setBackgroundColor(0, 0, 0);
  // viewer->addCoordinateSystem(1.0);
  // viewer->initCameraParameters();
  // // 设置点云颜色
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud(cloud_out, 255, 255, 255); // 白色
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_line(line_cloud, 255, 0, 0); // 红色

  // 添加点云到可视化对象中，并设置颜色
  // viewer->addPointCloud<pcl::PointXYZ>(cloud_out, color_cloud, "original_cloud");
  // viewer->addPointCloud<pcl::PointXYZ>(line_cloud, color_line, "line_cloud");
  // 计算拟合直线的斜率
  slope = coefficients->values[4] / coefficients->values[3];

  // 计算拟合直线的旋转角度（弧度）
  angle_new = atan(slope);

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

  // viewer->addLine(p1, p2, 0, 1, 0, "fitting_line");

  // 显示可视化窗口
  // while (!viewer->wasStopped()) {
      // viewer->spinOnce();
  // }s
  // pcl::io::savePCDFile("/home/mhj/Desktop/merged_cloud.pcd", *plane_removed_cloud);
  // pcl::io::savePCDFile("/home/mhj/Desktop/line_cloud.pcd", *line_cloud);
  

  ROS_INFO("===CALCULATED ANGLE: (%f)", (angle_new) * (180/pi_));

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

