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
  Desc: Grasp pose generation using Dex-Net, option to load images from a directory
        or subscribe to call the save_images service. To sample grasps the
        gqcnn_grasp service is called. This service returns the grasp candidates
        and the associated probabilities of success. Communication with MTC
        is achieved through an action service. This node contains the action client
        that sends a list grasp candidates and associated costs to MTC as feedback.

  PARAMETERS:
    load_images - load images from data directory
    image_dir - image data directory
    color_image_file (optional) - name of RGB image used as input to GQ-CNN
    depth_image_file (optional) - name of depth image used as input to GQ-CNN
    action_name - action namespace
    frame_id - frame of the grasp candidates sent to MTC
    trans_cam_opt - transform from camera link to optical link
    trans_base_cam - transform from robot base link to camera link
*/

// ROS
#include <ros/ros.h>

#include <moveit_task_constructor_dexnet/grasp_detection.h>

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("main", "Starting grasp_image_detection");
  ros::init(argc, argv, "grasp_image_detection");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_task_constructor_dexnet::GraspDetection grasp_detection(nh);

  ros::waitForShutdown();
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");

  return 0;
}
