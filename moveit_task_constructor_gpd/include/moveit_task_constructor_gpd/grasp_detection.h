/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik Inc.
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
   Desc:   Grasp pose detection (GPD) using point clouds
*/

// ROS
#include <sensor_msgs/PointCloud2.h>

// C++
#include <string>
#include <vector>
#include <functional>
#include <memory>

// Eigen
#include <Eigen/Geometry>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

// Action Server
#include <moveit_task_constructor_msgs/SampleGraspPosesAction.h>
#include <actionlib/server/simple_action_server.h>

namespace moveit_task_constructor_gpd
{
constexpr char LOGNAME[] = "grasp_pose_detection";

/**
* @brief Generates grasp poses for a generator stage with MTC
* @details Interfaces with the GPD lib using ROS messages and interfaces
*          with MTC using an Action Server
*/
class GraspDetection
{
public:
  /**
  * @brief Constructor
  * @param nh - node handle
  * @details loads parameters, registers callbacks for the action server,
             and initializes GPD
  */
  GraspDetection(const ros::NodeHandle& nh);

private:
  /**
  * @brief Loads parameters for action server, GPD, and relevant transformations
  */
  void loadParameters();

  /**
  * @brief Initialize action server callbacks and GPD
  * @details The point cloud (frame: panda_link0) is loaded from a file and
  *          the camera's origin relative to the point cloud is assumed to be at (0,0,0).
  */
  void init();

  /**
  * @brief Action server goal callback
  * @details Accepts goal from client and samples grasp candidates
  */
  void goalCallback();

  /**
  * @brief Preempt callback
  * @details Preempts goal
  */
  void preemptCallback();

  /**
  * @brief Samples grasp candidates using GPD
  * @details Compose grasp candidates, the candidates are sent back to the client
  *          using the feedback message. Only candidates with a positive grasp
  *          score are used. If there is at least one candidate with a positive
  *          score the result is set to success else it is a failure.
  */
  void sampleGrasps();

  /**
  * @brief Point cloud call back
  * @param msg - point cloud message
  * @details Segments objects from table plane
  */
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
  ros::NodeHandle nh_;         // node handle
  ros::Subscriber cloud_sub_;  // subscribes to point cloud
  ros::Publisher cloud_pub_;   // publishes segmented cloud

  std::unique_ptr<actionlib::SimpleActionServer<moveit_task_constructor_msgs::SampleGraspPosesAction>>
      server_;                                                       // action server
  moveit_task_constructor_msgs::SampleGraspPosesFeedback feedback_;  // action feedback message
  moveit_task_constructor_msgs::SampleGraspPosesResult result_;      // action result message

  std::string path_to_pcd_file_;    // path to cylinder pcd file
  std::string path_to_gpd_config_;  // path to GPD config file
  std::string point_cloud_topic_;   // point cloud topic name
  std::string goal_name_;           // action name
  std::string action_name_;         // action namespace
  std::string frame_id_;            // frame of point cloud/grasps

  bool goal_active_;  // action goal status
  bool load_cloud_;   // load cloud from file

  std::vector<double> view_point_;                      // origin of the camera
  std::unique_ptr<gpd::GraspDetector> grasp_detector_;  // used to run the GPD algorithm
  std::unique_ptr<gpd::util::Cloud> cloud_camera_;      // stores point cloud with (optional) camera information

  Eigen::Isometry3d trans_base_cam_;     // transformation from base link to camera link
  Eigen::Isometry3d transform_cam_opt_;  // transformation from camera link to optical link
};
}  // namespace moveit_task_constructor_gpd
