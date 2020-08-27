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
   Desc:   Samples grasp candidates using GQ-CNN and Dex-Net model weights/data sets
*/

// C++
#include <string>
#include <memory>

// Eigen
#include <Eigen/Geometry>

// Action Server
#include <moveit_task_constructor_msgs/SampleGraspPosesAction.h>
#include <actionlib/server/simple_action_server.h>

namespace moveit_task_constructor_dexnet
{
constexpr char LOGNAME[] = "grasp_image_detection";

/**
* @brief Generates grasp poses for a generator stage with MTC
* @details Interfaces with the GQ-CNN and Dex-Net data sets using ROS messages
*          and interfaces with MTC using an Action Server.
*/
class GraspDetection
{
public:
  /**
  * @brief Constructor
  * @param nh - node handle
  * @details loads parameter and registers callbacks for the action server
  */
  GraspDetection(const ros::NodeHandle& nh);

private:
  /**
  * @brief Loads parameters for action server, image data, and relevant transformations
  */
  void loadParameters();

  /**
  * @brief Initialize action server callbacks and Image service client (optional)
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
  * @brief Requests images by calling the save_images service
  * @details If Images are not loaded from the data directory they need to be
  *          requested. This assumes a camera is publishing the images.
  */
  void requestImages();

  /**
  * @brief Samples grasp candidates using a GQ-CNN and Dex-Net model weights/data sets
  * @details Compose grasp candidates, the candidates are sent back to the client
  *          using the feedback message. The calls the gqcnn_grasp service to
  *          execute the learned policy.
  */
  void sampleGrasps();

private:
  ros::NodeHandle nh_;               // node handle
  ros::ServiceClient gqcnn_client_;  // gqcnn service client
  ros::ServiceClient image_client_;  // image saving service client

  std::unique_ptr<actionlib::SimpleActionServer<moveit_task_constructor_msgs::SampleGraspPosesAction>>
      server_;                                                       // action server
  moveit_task_constructor_msgs::SampleGraspPosesFeedback feedback_;  // action feedback message
  moveit_task_constructor_msgs::SampleGraspPosesResult result_;      // action result message

  std::string goal_name_;    // action name
  std::string action_name_;  // action namespace
  std::string frame_id_;     // frame of point cloud/grasps

  std::string image_dir_;         // directory images saved
  std::string color_image_file_;  // file path to color image
  std::string depth_image_file_;  // file path to depth image

  bool load_images_;  // load images from file

  Eigen::Isometry3d trans_base_cam_;     // transformation from base link to camera link
  Eigen::Isometry3d transform_cam_opt_;  // transformation from camera link to optical link
};
}  // namespace moveit_task_constructor_dexnet
