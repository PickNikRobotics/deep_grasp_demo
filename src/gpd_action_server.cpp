/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

 /* Author: Boston Cleek
    Desc:   GPD action server
 */

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <pcl_conversions/pcl_conversions.h>

// C++
#include <functional>
#include <memory>
#include <vector>
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// Eigen
#include <Eigen/Dense>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>
#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_geometry.h>

// Action Server
#include <moveit_task_constructor_msgs/GenerateDeepGraspPoseAction.h>
#include <actionlib/server/simple_action_server.h>


constexpr char LOGNAME[] = "gpd_action_server";

// TODO: Interface directly with gpd lib
// TODO: Option to use pcd file or sensor data

namespace gpd_action_server
{
class GraspAction
{
public:
  GraspAction(const ros::NodeHandle &nh) : nh_(nh)
  {
    loadParameters();
    init();
  }

  void loadParameters()
  {
    ROS_INFO_NAMED(LOGNAME, "Loading grasp action server parameters");
  	ros::NodeHandle pnh("~");

    size_t errors = 0;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "path_to_pcd_file", path_to_pcd_file_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "path_to_gpd_config", path_to_gpd_config_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "action_name", action_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "view_point", view_point_);
    rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

    ROS_INFO("Path to PCD file: %s", path_to_pcd_file_.c_str());
    ROS_INFO("Path to GPD file: %s", path_to_gpd_config_.c_str());

  }

  void init()
  {
    // action server
    server_.reset(new actionlib::SimpleActionServer<moveit_task_constructor_msgs::GenerateDeepGraspPoseAction>(nh_, action_name_, false));
    server_->registerGoalCallback(std::bind(&GraspAction::goalCallback, this));
    server_->registerPreemptCallback(std::bind(&GraspAction::preemptCallback, this));
    server_->start();

    // GPD point cloud camera
    // load cloud from file
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    // gpd::util::PointCloudRGB::Ptr cloud(new gpd::util::PointCloudRGB);


    gpd::util::PointCloudPointNormal::Ptr cloud(new gpd::util::PointCloudPointNormal);

    // ROS_INFO("Loading cloud from file...");
    // if (pcl::io::loadPCDFile(path_to_pcd_file_, *cloud.get()) == -1){
    //   ROS_ERROR("Failed to load cloud");
    // }
    // else{
    //   ROS_INFO("Cloud loaded size: %lu", cloud->points.size());
    // }

    // set camera view origin
    // assume cloud was taken using one camera
    // Eigen::Vector3d temp_mat;
    // temp_mat << view_point_.at(0), view_point_.at(1), view_point_.at(2);
    Eigen::Matrix3Xd camera_view_point(3,1);
    // camera_view_point.col(0) = temp_mat;

    cloud_camera_.reset(new gpd::util::Cloud(cloud, 0, camera_view_point));

    // gpd::util::Cloud cloud_cam(path_to_pcd_file_, camera_view_point);

    // gpd::util::Cloud cloud_cam(cloud, 0, camera_view_point);

    // Grasp detector
    // gpd::GraspDetector grasp_d(path_to_gpd_config_);


  }



  void goalCallback()
  {
    // goal_ = server_.acceptNewGoal()->action_name;
    // ROS_INFO("New goal accepted: %s", goal_.c_str());
    //
    // ROS_INFO("Loading cloud from file...");
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //
    // if (!pcl::io::loadPCDFile(path_to_pcd_file, *cloud.get()))
    // {
    //   ROS_INFO("Cloud loaded size: %lu", cloud->points.size());
    // }
    //
    // else
    // {
    //   ROS_ERROR("Failed to load cloud");
    // }
    //
    // sensor_msgs::PointCloud2 cloud_msg;
    // pcl::toROSMsg(*cloud.get(), cloud_msg);
    //
    // // TODO: load this as a param
    // cloud_msg.header.frame_id = "panda_link0";
    //
    // gpd_cloud_pub_.publish(cloud_msg);
  }


  void preemptCallback()
  {
    // ROS_INFO("Preempted %s:", goal_.c_str());
    // server_.setPreempted();
  }


  // void graspCallBack(const gpd_ros::GraspConfigList::ConstPtr &msg)
  // {
  //   ROS_INFO("Grasp server received %lu grasp candidates", msg->grasps.size());
  //   result_.grasp_candidates.resize(msg->grasps.size());
  //   result_.scores.resize(msg->grasps.size());
  //
  //   for(unsigned int i = 0; i < msg->grasps.size(); i++)
  //   {
  //     result_.grasp_candidates.at(i).header.frame_id = msg->header.frame_id;
  //     result_.grasp_candidates.at(i).pose.position = msg->grasps.at(i).position;
  //     result_.grasp_candidates.at(i).pose.orientation = msg->grasps.at(i).orientation;
  //
  //     result_.scores.at(i) = msg->grasps.at(i).score.data;
  //   }
  //
  //   server_.setSucceeded(result_);
  // }

private:
  ros::NodeHandle nh_;

  std::unique_ptr<actionlib::SimpleActionServer<moveit_task_constructor_msgs::GenerateDeepGraspPoseAction>> server_;
  moveit_task_constructor_msgs::GenerateDeepGraspPoseFeedback feedback_;
  moveit_task_constructor_msgs::GenerateDeepGraspPoseResult result_;

  std::string path_to_pcd_file_;
  std::string path_to_gpd_config_;
  std::string action_name_;

  std::vector<double> view_point_;           // origin of the camera

  // gpd::GraspDetector grasp_detector_;     // used to run the GPD algorithm
  std::unique_ptr<gpd::util::Cloud> cloud_camera_;            // stores point cloud with (optional) camera information

};
}



int main(int argc, char** argv) {
	ROS_INFO_NAMED(LOGNAME, "Init gpd_action_server");
	ros::init(argc, argv, "gpd_server");
  ros::NodeHandle nh;

  gpd_action_server::GraspAction grasp_action(nh);

  // ros::spin();

	return 0;
}
