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
   Desc:   GQCNN action server
*/

// ROS
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometry_msgs/PoseStamped.h>

// C++
// #include <functional>
#include <memory>

// Action Server
#include <moveit_task_constructor_msgs/GenerateDeepGraspPoseAction.h>
#include <actionlib/server/simple_action_server.h>

#include <gqcnn_demo/image_server.h>
#include <gqcnn_demo/GQCNNGrasp.h>




constexpr char LOGNAME[] = "gqcnn_action_server";


namespace gqcnn_demo
{
class GraspAction
{
public:
  GraspAction(const ros::NodeHandle& nh) : nh_(nh), q_val_(0.0)
  {
    loadParameters();
    init();
  }

  void loadParameters()
  {

  }

  void init()
  {
    gqcnn_client_ = nh_.serviceClient<gqcnn_demo::GQCNNGrasp>("gqcnn_grasp");

    gqcnn_demo::GQCNNGrasp grasp_srv;
    grasp_srv.request.name = "gqcnn";

    ros::service::waitForService("gqcnn_grasp", ros::Duration(3.0));

    if(gqcnn_client_.call(grasp_srv)){
      ROS_INFO_NAMED(LOGNAME, "Called gqcnn_grasp service, waiting for response...");
      grasp_ = grasp_srv.response.grasp;
      q_val_ = grasp_srv.response.q_val;
      ROS_INFO_NAMED(LOGNAME, "Results recieved");
    }

    else{
      ROS_WARN_NAMED(LOGNAME, "Failed to call gqcnn_grasp service");
    }
  }


private:
  ros::NodeHandle nh_;
  ros::ServiceClient gqcnn_client_;

  geometry_msgs::PoseStamped grasp_;
  double q_val_;


};
} // namespace gqcnn_demo



int main(int argc, char** argv)
{
  ROS_INFO_NAMED(LOGNAME, "Init gqcnn_action_server");
  ros::init(argc, argv, "gqcnn_server");
  ros::NodeHandle nh;

  // TODO: add async spinner

  // gqcnn_demo::ImageServer image_server(nh);
  gqcnn_demo::GraspAction grasp_action(nh);
  ros::spin();

  return 0;
}











//
