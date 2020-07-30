/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *this
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE
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
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/PointCloud2.h>

// C++
#include <functional>
#include <utility>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Dense>

// GPD
#include <gpd_ros/GraspConfigList.h>

// Action Server
#include <actionlib/server/simple_action_server.h>
#include <moveit_task_constructor_msgs/GenerateDeepGraspPoseAction.h>

constexpr char LOGNAME[] = "gpd_action_server";
static std::string PATH_TO_PCD_FILE;
static std::string CLOUD_FRAME_ID;
// static geometry_msgs::PoseStamped grasp1;

namespace gpd_action_server
{
/**
* @brief Generates grasp poses for a generator stage with MTC
* @details Interfaces with the GPD lib using ROS messages and interfaces
*          with MTC using a Action Server
*/
class GraspAction
{
public:
  /**
  * @brief Constructor
  * @param nh - node handle
  * @param action_name - action namespace
  * @details Registers callbacks for the action server and for gpd_ros
  */
  GraspAction(ros::NodeHandle& nh, std::string action_name) : nh_(nh), server_(nh_, std::move(action_name), false)
  {
    server_.registerGoalCallback(std::bind(&GraspAction::goalCallback, this));
    server_.registerPreemptCallback(std::bind(&GraspAction::preemptCallback, this));
    server_.start();

    deep_grasp_sub_ = nh_.subscribe("detect_grasps/clustered_grasps", 1, &GraspAction::graspCallBack, this);
    gpd_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_stitched", 1, true);
  }

  /**
  * @brief Action server goal callback
  * @details Accepts goal from client, loads point cloud, and sends cloud to
  * gpd_ros
  */
  void goalCallback()
  {
    goal_name_ = server_.acceptNewGoal()->action_name;
    ROS_INFO_NAMED(LOGNAME, "New goal accepted: %s", goal_name_.c_str());

    ROS_INFO_NAMED(LOGNAME, "Loading cloud from file...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (!pcl::io::loadPCDFile(PATH_TO_PCD_FILE, *cloud.get()))
    {
      ROS_INFO_NAMED(LOGNAME, "Cloud loaded size: %lu", cloud->points.size());
    }

    else
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to load cloud");
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud.get(), cloud_msg);

    cloud_msg.header.frame_id = CLOUD_FRAME_ID;

    gpd_cloud_pub_.publish(cloud_msg);

    // // send hard coded feedback/result for testing
    // feedback_.grasp_candidates.resize(1);
    // feedback_.costs.resize(1);
    //
    // feedback_.grasp_candidates.at(0) = grasp1;
    // feedback_.costs.at(0) = 0.0;
    //
    // server_.publishFeedback(feedback_);
    //
    // result_.grasp_state = "success";
    // server_.setSucceeded(result_);
  }

  /**
  * @brief Preempt callback
  * @details Preempts goal
  */
  void preemptCallback()
  {
    ROS_INFO_NAMED(LOGNAME, "Preempted %s:", goal_name_.c_str());
    server_.setPreempted();
  }

  /**
  * @brief Grasp callback
  * @param msg - Grasp configurations from gpd_ros
  * @details Receives grasps from gpd_ros and sends feedback/results to MTC
  * action server
  */
  void graspCallBack(const gpd_ros::GraspConfigList::ConstPtr& msg)
  {
    ROS_INFO_NAMED(LOGNAME, "Grasp server received %lu grasp candidates", msg->grasps.size());

    // Do not use grasps with score < 0
    // Grasp is selected based on cost not score
    // Invert score to represent grasp with lowest cost
    for (unsigned int i = 0; i < msg->grasps.size(); i++)
    {
      if (msg->grasps.at(i).score.data > 0.0)
      {
        geometry_msgs::PoseStamped grasp;
        grasp.header.frame_id = msg->header.frame_id;
        grasp.pose.position = msg->grasps.at(i).position;
        grasp.pose.orientation = msg->grasps.at(i).orientation;

        feedback_.grasp_candidates.emplace_back(grasp);
        feedback_.costs.emplace_back(static_cast<double>(1.0 / msg->grasps.at(i).score.data));
      }
    }

    server_.publishFeedback(feedback_);

    result_.grasp_state = "success";
    server_.setSucceeded(result_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber deep_grasp_sub_;
  ros::Publisher gpd_cloud_pub_;

  std::string goal_name_;
  actionlib::SimpleActionServer<moveit_task_constructor_msgs::GenerateDeepGraspPoseAction> server_;
  moveit_task_constructor_msgs::GenerateDeepGraspPoseFeedback feedback_;
  moveit_task_constructor_msgs::GenerateDeepGraspPoseResult result_;
};
}  // namespace gpd_action_server

int main(int argc, char** argv)
{
  ROS_INFO_NAMED(LOGNAME, "Init gpd_action_server");
  ros::init(argc, argv, "gpd_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string action_name;

  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "path_to_pcd_file", PATH_TO_PCD_FILE);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "action_name", action_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "cloud_frame_id", CLOUD_FRAME_ID);

  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
  ROS_INFO_NAMED(LOGNAME, "Path to PCD file: %s", PATH_TO_PCD_FILE.c_str());

  // // hard code grasp pose for now
  // grasp1.header.frame_id = "object";
  // grasp1.pose.position.x = 0.0;
  // grasp1.pose.position.y = 0.0;
  // grasp1.pose.position.z = 0.0;
  // grasp1.pose.orientation.w = 1.0;

  gpd_action_server::GraspAction grasp_action(nh, action_name);
  ros::spin();

  return 0;
}
