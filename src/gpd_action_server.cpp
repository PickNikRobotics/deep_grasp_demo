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

// C++
#include <functional>
#include <vector>

// #include <rosparam_shortcuts/rosparam_shortcuts.h>

// Action Server
#include <moveit_task_constructor_msgs/GenerateDeepGraspPoseAction.h>
#include <actionlib/server/simple_action_server.h>

constexpr char LOGNAME[] = "gpd_action_server";
static geometry_msgs::PoseStamped grasp1, grasp2;


class GraspAction
{
public:
  GraspAction(std::string server_name) : server_(nh_, server_name, false)
  {
    server_.registerGoalCallback(std::bind(&GraspAction::goalCallback, this));
    server_.registerPreemptCallback(std::bind(&GraspAction::preemptCallback, this));

    // start server
    server_.start();
  }


  void goalCallback()
  {
    // accept a new goal
    goal_ = server_.acceptNewGoal()->action_name;
    ROS_INFO_NAMED(LOGNAME, "New goal accepted: %s", goal_.c_str());

    ROS_INFO_NAMED(LOGNAME, "Succeeded");
    result_.grasp_candidates.resize(2);
    result_.grasp_candidates.at(0) = grasp1;
    result_.grasp_candidates.at(1) = grasp2;
    server_.setSucceeded(result_);
  }


  void preemptCallback()
  {
    // set the action state to preempted
    ROS_INFO_NAMED(LOGNAME, "Preempted %s:", goal_.c_str());
    server_.setPreempted();
  }

private:
  ros::NodeHandle nh_;
  std::string goal_;
  actionlib::SimpleActionServer<moveit_task_constructor_msgs::GenerateDeepGraspPoseAction> server_;
  moveit_task_constructor_msgs::GenerateDeepGraspPoseFeedback feedback_;
  moveit_task_constructor_msgs::GenerateDeepGraspPoseResult result_;
};




int main(int argc, char** argv) {
	ROS_INFO_NAMED(LOGNAME, "Init gpd_action_server");
	ros::init(argc, argv, "gpd_server");

  std::string server_name = "sample_grasps";

  // hard code grasp pose for now
  grasp1.header.frame_id = "object";
  grasp1.pose.position.x = 0.0;
  grasp1.pose.position.y = 0.0;
  grasp1.pose.position.z = 0.0;
  grasp1.pose.orientation.w = 1.0;

  grasp2 = grasp1;
  grasp2.pose.position.z = 0.0;
  grasp2.pose.orientation.z = 0.259;
  grasp2.pose.orientation.w = 0.966;

  GraspAction grasp_action(server_name);
  ros::spin();

	// ros::waitForShutdown();
	return 0;
}
