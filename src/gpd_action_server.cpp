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
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// Eigen
#include <Eigen/Dense>

// GPD
#include <gpd_ros/GraspConfigList.h>

// Action Server
#include <moveit_task_constructor_msgs/GenerateDeepGraspPoseAction.h>
#include <actionlib/server/simple_action_server.h>


constexpr char LOGNAME[] = "gpd_action_server";
static std::string path_to_pcd_file;
// static geometry_msgs::PoseStamped grasp1, grasp2;

// TODO: Interface directly with gpd lib
// TODO: Option to use pcd file or sensor data

namespace gpd_action_server
{
class GraspAction
{
public:
  GraspAction(const ros::NodeHandle& nh, std::string server_name)
       : nh_(nh), server_(nh_, server_name, false)
  {
    server_.registerGoalCallback(std::bind(&GraspAction::goalCallback, this));
    server_.registerPreemptCallback(std::bind(&GraspAction::preemptCallback, this));
    server_.start();

    deep_grasp_sub_ = nh_.subscribe("detect_grasps/clustered_grasps", 1, &GraspAction::graspCallBack, this);
    gpd_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_stitched", 1, true);
  }


  void goalCallback()
  {
    goal_ = server_.acceptNewGoal()->action_name;
    ROS_INFO("New goal accepted: %s", goal_.c_str());

    ROS_INFO("Loading cloud from file...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (!pcl::io::loadPCDFile(path_to_pcd_file, *cloud.get()))
    {
      ROS_INFO("Cloud loaded size: %lu", cloud->points.size());
    }

    else
    {
      ROS_ERROR("Failed to load cloud");
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud.get(), cloud_msg);

    // TODO: load this as a param
    cloud_msg.header.frame_id = "panda_link0";

    gpd_cloud_pub_.publish(cloud_msg);


    // send hard coded result for testing
    // result_.grasp_candidates.resize(2);
    // result_.grasp_candidates.at(0) = grasp1;
    // result_.grasp_candidates.at(1) = grasp2;
    // server_.setSucceeded(result_);
  }


  void preemptCallback()
  {
    ROS_INFO("Preempted %s:", goal_.c_str());
    server_.setPreempted();
  }


  void graspCallBack(const gpd_ros::GraspConfigList::ConstPtr &msg)
  {
    ROS_INFO("Grasp server received %lu grasp candidates", msg->grasps.size());
    result_.grasp_candidates.resize(msg->grasps.size());
    result_.scores.resize(msg->grasps.size());

    for(unsigned int i = 0; i < msg->grasps.size(); i++)
    {
      result_.grasp_candidates.at(i).header.frame_id = msg->header.frame_id;
      result_.grasp_candidates.at(i).pose.position = msg->grasps.at(i).position;
      result_.grasp_candidates.at(i).pose.orientation = msg->grasps.at(i).orientation;

      result_.scores.at(i) = msg->grasps.at(i).score.data;
    }

    server_.setSucceeded(result_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber deep_grasp_sub_;
  ros::Publisher gpd_cloud_pub_;

  std::string goal_;
  actionlib::SimpleActionServer<moveit_task_constructor_msgs::GenerateDeepGraspPoseAction> server_;
  moveit_task_constructor_msgs::GenerateDeepGraspPoseFeedback feedback_;
  moveit_task_constructor_msgs::GenerateDeepGraspPoseResult result_;
};
}



int main(int argc, char** argv) {
	ROS_INFO_NAMED(LOGNAME, "Init gpd_action_server");
	ros::init(argc, argv, "gpd_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "path_to_pcd_file", path_to_pcd_file);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
  ROS_INFO("Path to PCD file: %s", path_to_pcd_file.c_str());

  std::string server_name = "sample_grasps";

  // // hard code grasp pose for now
  // grasp1.header.frame_id = "object";
  // grasp1.pose.position.x = 0.0;
  // grasp1.pose.position.y = 0.0;
  // grasp1.pose.position.z = 0.0;
  // grasp1.pose.orientation.w = 1.0;
  //
  // grasp2 = grasp1;
  // grasp2.pose.position.z = 0.0;
  // grasp2.pose.orientation.z = 0.259;
  // grasp2.pose.orientation.w = 0.966;

  gpd_action_server::GraspAction grasp_action(nh, server_name);
  ros::spin();

	return 0;
}
