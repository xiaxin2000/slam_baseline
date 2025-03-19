/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 Localization and mapping program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>
#include <autoware_config_msgs/ConfigNDTMapping.h>
#include <autoware_config_msgs/ConfigNDTMappingOutput.h>

#include <time.h>

#include <iostream>

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

// global variables
static pose previous_pose, guess_pose, current_pose,ndt_pose, added_pose, localizer_pose;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;  // current_pose - previous_pose

static pcl::PointCloud<pcl::PointXYZI> map;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

// Default values
static std::string pcd_map_filename = "/home/carma/CEE_298_Mapping/map.pcd";
static int max_iter = 30;        // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.1;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon
static double min_add_scan_shift = 3.0;
// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
    t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_map_pub;
static ros::Publisher current_pose_pub;
static geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();
static double min_scan_range = 5.0;
static double max_scan_range = 200.0;


static Eigen::Matrix4f tf_btol, tf_ltob;


static double fitness_score;
static bool has_converged;
static int final_num_iteration;
static double transformation_probability;

static void output_callback(const std_msgs::Bool::ConstPtr& input)
{
  double filter_res = 0.3;
  std::cout << "output_callback" << std::endl;
  std::cout << "filter_res: " << filter_res << std::endl;
  std::cout << "pcd_map_filename: " << pcd_map_filename << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  map_ptr->header.frame_id = "map";
  map_filtered->header.frame_id = "map";
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);

  // Apply voxelgrid filter
  if (filter_res == 0.0)
  {
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  }
  else
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
    voxel_grid_filter.setInputCloud(map_ptr);
    voxel_grid_filter.filter(*map_filtered);
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    std::cout << "Filtered: " << map_filtered->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_filtered, *map_msg_ptr);
  }

  ndt_map_pub.publish(*map_msg_ptr);

  // Writing Point Cloud data to PCD file
  if (filter_res == 0.0)
  {
    int ret = pcl::io::savePCDFileBinary(pcd_map_filename, *map_ptr);
    std::cout << "Saved " << map_ptr->points.size() << " data points to " << pcd_map_filename << "." << std::endl;
  }
  else
  {
    int ret = pcl::io::savePCDFileBinary(pcd_map_filename, *map_filtered);
    std::cout << "Saved " << map_filtered->points.size() << " data points to " << pcd_map_filename << "." << std::endl;
  }
}


static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}


static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  pcl::fromROSMsg(*input, tmp);

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (min_scan_range < r && r < max_scan_range)
    {
      scan.push_back(p);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
  if (initial_scan_loaded == 0)
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    map += *transformed_scan_ptr;
    initial_scan_loaded = 1;
  }
  std::cout<<" map size:"<<map.points.size()<<std::endl;
  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

  ndt.setTransformationEpsilon(trans_eps);
  ndt.setStepSize(step_size);
  ndt.setResolution(ndt_res);
  ndt.setMaximumIterations(max_iter);
  ndt.setInputSource(filtered_scan_ptr);

  static bool is_first_map = true;
  if (is_first_map == true)
  {
    ndt.setInputTarget(map_ptr);
    is_first_map = false;
  }

  guess_pose.x = previous_pose.x + diff_x;
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;

  pose guess_pose_for_ndt;

  guess_pose_for_ndt = guess_pose;

  Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;

  t3_end = ros::Time::now();
  d3 = t3_end - t3_start;

  t4_start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  ndt.align(*output_cloud, init_guess);
  fitness_score = ndt.getFitnessScore();
  t_localizer = ndt.getFinalTransformation();
  has_converged = ndt.hasConverged();
  final_num_iteration = ndt.getFinalNumIteration();
  transformation_probability = ndt.getTransformationProbability();

  t_base_link = t_localizer * tf_ltob;

  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_l, mat_b;

  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update localizer_pose.
  localizer_pose.x = t_localizer(0, 3);
  localizer_pose.y = t_localizer(1, 3);
  localizer_pose.z = t_localizer(2, 3);
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  std::cout<<"localizer_pose.x: "<<localizer_pose.x<<std::endl;
  std::cout<<"localizer_pose.y: "<<localizer_pose.y<<std::endl;
  std::cout<<"localizer_pose.z: "<<localizer_pose.z<<std::endl;

  // Update ndt_pose.
  ndt_pose.x = t_base_link(0, 3);
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

  current_pose.x = ndt_pose.x;
  current_pose.y = ndt_pose.y;
  current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;
  current_pose.yaw = ndt_pose.yaw;

  std::cout<<" ********************************************"<<std::endl;
  std::cout<<" current_pose.x:"<<current_pose.x<<std::endl;
  std::cout<<" current_pose.y:"<<current_pose.y<<std::endl;
  std::cout<<" ********************************************"<<std::endl;

  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "laser"));

  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();
  std::cout<<"secs: "<<secs<<std::endl;

  // Calculate the offset (curren_pos - previous_pos)
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  std::cout<<"diff_x: "<<diff_x<<std::endl;
  std::cout<<"diff_y: "<<diff_y<<std::endl;

  // Update position and posture. current_pos -> previous_pos
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  // Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift)
  {
    map += *transformed_scan_ptr;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;

    ndt.setInputTarget(map_ptr);
  }

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  ndt_map_pub.publish(*map_msg_ptr);

  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  current_pose_msg.pose.position.x = current_pose.x;
  current_pose_msg.pose.position.y = current_pose.y;
  current_pose_msg.pose.position.z = current_pose.z;
  current_pose_msg.pose.orientation.x = q.x();
  current_pose_msg.pose.orientation.y = q.y();
  current_pose_msg.pose.orientation.z = q.z();
  current_pose_msg.pose.orientation.w = q.w();

  current_pose_pub.publish(current_pose_msg);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map.points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_num_iteration << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

int main(int argc, char** argv)
{
  previous_pose.x = 0.0;
  previous_pose.y = 0.0;
  previous_pose.z = 0.0;
  previous_pose.roll = 0.0;
  previous_pose.pitch = 0.0;
  previous_pose.yaw = 0.0;

  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;

  current_pose.x = 0.0;
  current_pose.y = 0.0;
  current_pose.z = 0.0;
  current_pose.roll = 0.0;
  current_pose.pitch = 0.0;
  current_pose.yaw = 0.0;

  guess_pose.x = 0.0;
  guess_pose.y = 0.0;
  guess_pose.z = 0.0;
  guess_pose.roll = 0.0;
  guess_pose.pitch = 0.0;
  guess_pose.yaw = 0.0;

  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;

  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;


  ros::init(argc, argv, "ndt_mapping");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set log file name.
  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm* pnow = std::localtime(&now);
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);

  // setting parameters
  private_nh.getParam("pcd_map_filename", pcd_map_filename);
  private_nh.getParam("max_iter", max_iter);
  private_nh.getParam("ndt_res", ndt_res);
  private_nh.getParam("step_size", step_size);
  private_nh.getParam("trans_eps", trans_eps);
  private_nh.getParam("min_add_scan_shift", min_add_scan_shift);
  private_nh.getParam("voxel_leaf_size", voxel_leaf_size);

  std::string lidar_frame;
  nh.param("localizer", lidar_frame, std::string("rslidar"));
  tf::TransformListener tf_listener;
  tf::StampedTransform tf_baselink2primarylidar;
  try
  {
    tf_listener.waitForTransform("base_link", lidar_frame, ros::Time(), ros::Duration(1.0));
    tf_listener.lookupTransform("base_link", lidar_frame, ros::Time(), tf_baselink2primarylidar);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("Query base_link to primary lidar frame through TF tree failed: %s", ex.what());

    // fall back to ros parameter for the transform
    std::vector<double> bl2pl_vec;
    if (!nh.getParam("tf_baselink2primarylidar", bl2pl_vec))
    {
      std::cout << "ros parameter tf_baselink2primarylidar is not set." << std::endl;
      return 1;
    }

    // translation x, y, z, yaw, pitch, and roll
    if (bl2pl_vec.size() != 6)
    {
      std::cout << "ros parameter tf_baselink2primarylidar is not valid." << std::endl;
      return 1;
    }
    ROS_WARN("Query through ros parameter tf_baselink2primarylidar succeeded.");

    tf::Vector3 trans(bl2pl_vec[0], bl2pl_vec[1], bl2pl_vec[2]);
    tf::Quaternion quat;
    quat.setRPY(bl2pl_vec[5], bl2pl_vec[4], bl2pl_vec[3]);
    tf_baselink2primarylidar.setOrigin(trans);
    tf_baselink2primarylidar.setRotation(quat);
  }

  Eigen::Affine3d tf_affine;
  tf::transformTFToEigen(tf_baselink2primarylidar, tf_affine);
  tf_btol = tf_affine.matrix().cast<float>();
  tf_ltob = tf_btol.inverse();

  std::cout << "tf_baselink2primarylidar: \n" << tf_btol << std::endl;

  map.header.frame_id = "map";

  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

  ros::Subscriber output_sub = nh.subscribe("savemap", 10, output_callback);
  ros::Subscriber points_sub = nh.subscribe("/rslidar_points", 100000, points_callback);

  ros::spin();

  return 0;
}
