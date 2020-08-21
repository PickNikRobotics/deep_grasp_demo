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
   Desc:   GQCNN action server

   PARAMETERS:
   PUBLISHES:
   SUBSCRIBES:
   SERVICES:
*/

// ROS
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometry_msgs/PoseStamped.h>

// C++
#include <memory>
#include <vector>

// Eigen
#include <Eigen/Geometry>

// Action Server
#include <moveit_task_constructor_msgs/SampleGraspPosesAction.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit_task_constructor_dexnet/GQCNNGrasp.h>
#include <moveit_task_constructor_dexnet/Images.h>

constexpr char LOGNAME[] = "grasp_image_detection";


namespace moveit_task_constructor_dexnet
{
class GraspDetection
{
public:
  GraspDetection(const ros::NodeHandle& nh) : nh_(nh), q_val_(0.0)
  {
    loadParameters();
    init();
  }

  void loadParameters()
  {
    ROS_INFO_NAMED(LOGNAME, "Loading grasp action server parameters");
    ros::NodeHandle pnh("~");

    size_t errors = 0;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "load_images", load_images_);
    if(load_images_){
      errors += !rosparam_shortcuts::get(LOGNAME, pnh, "color_image_file", color_image_file_);
      errors += !rosparam_shortcuts::get(LOGNAME, pnh, "depth_image_file", depth_image_file_);
    }

    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "action_name", action_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "frame_id", frame_id_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "image_dir", image_dir_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "trans_cam_opt", transform_cam_opt_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "trans_base_cam", trans_base_cam_);
    rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
  }

  void init()
  {
    // action server
    server_.reset(new actionlib::SimpleActionServer<moveit_task_constructor_msgs::SampleGraspPosesAction>(
        nh_, action_name_, false));
    server_->registerGoalCallback(std::bind(&GraspDetection::goalCallback, this));
    server_->registerPreemptCallback(std::bind(&GraspDetection::preemptCallback, this));
    server_->start();

    // gqcnn service
    gqcnn_client_ = nh_.serviceClient<moveit_task_constructor_dexnet::GQCNNGrasp>("gqcnn_grasp");
    ros::service::waitForService("gqcnn_grasp", ros::Duration(3.0));

    // not loading images from file so need to call server to save them
    if(!load_images_){
      color_image_file_ = "rgb_object.png";
      depth_image_file_= "depth_object.png";

      image_client_ = nh_.serviceClient<moveit_task_constructor_dexnet::Images>("save_images");
      ros::service::waitForService("save_images", ros::Duration(3.0));
    }
  }


  void goalCallback()
  {
    goal_name_ = server_->acceptNewGoal()->action_name;
    ROS_INFO_NAMED(LOGNAME, "New goal accepted: %s", goal_name_.c_str());

    // save images
    if(!load_images_){
      requestImages();
    }

    sampleGrasps();
  }


  void preemptCallback()
  {
    ROS_INFO_NAMED(LOGNAME, "Preempted %s:", goal_name_.c_str());
    server_->setPreempted();
  }


  void requestImages()
  {
    moveit_task_constructor_dexnet::Images image_srv;
    image_srv.request.color_file = color_image_file_;
    image_srv.request.depth_file = depth_image_file_;

    if(image_client_.call(image_srv)){
      ROS_INFO_NAMED(LOGNAME, "Called save_images service, waiting for response...");
      if (image_srv.response.color_saved && image_srv.response.color_saved){
        ROS_INFO_NAMED(LOGNAME, "Images saved");
      } else{
        ROS_ERROR_NAMED(LOGNAME, "Images not saved");
      }
    } else{
      ROS_ERROR_NAMED(LOGNAME, "Failed to call save_images service");
    }
  }


  void sampleGrasps()
  {
    moveit_task_constructor_dexnet::GQCNNGrasp grasp_srv;
    grasp_srv.request.color_img_file_path = image_dir_ + color_image_file_;
    grasp_srv.request.depth_img_file_path = image_dir_ + depth_image_file_;


    if(gqcnn_client_.call(grasp_srv)){
      ROS_INFO_NAMED(LOGNAME, "Called gqcnn_grasp service, waiting for response...");
      grasp_ = grasp_srv.response.grasp;
      q_val_ = grasp_srv.response.q_val;
      ROS_INFO_NAMED(LOGNAME, "Results recieved");

      // transform grasp from camera optical link into frame_id (panda_link0)
      // the demo is using fake_controllers there is no tf data for the camera
      // convert PoseStamped to transform (optical link to grasp)
      const Eigen::Isometry3d transform_opt_grasp = Eigen::Translation3d(grasp_.pose.position.x,
                                                                   grasp_.pose.position.y,
                                                                   grasp_.pose.position.z) *
                                                Eigen::Quaterniond(grasp_.pose.orientation.w,
                                                                   grasp_.pose.orientation.x,
                                                                   grasp_.pose.orientation.y,
                                                                   grasp_.pose.orientation.z);

      // the 6dof grasp pose in frame_id (panda_link0)
      // TODO compose this T once in init()
      const Eigen::Isometry3d transform_base_grasp = trans_base_cam_ * transform_cam_opt_ * transform_opt_grasp;
      const Eigen::Vector3d trans = transform_base_grasp.translation();
      const Eigen::Quaterniond rot(transform_base_grasp.rotation());

      // TODO grasp_ should not be a memeber
      // convert back to PoseStamped
      grasp_.header.frame_id = frame_id_;
      grasp_.pose.position.x = trans.x();
      grasp_.pose.position.y = trans.y();
      grasp_.pose.position.z = trans.z();

      grasp_.pose.orientation.w = rot.w();
      grasp_.pose.orientation.x = rot.x();
      grasp_.pose.orientation.y = rot.y();
      grasp_.pose.orientation.z = rot.z();

      // grasp_.pose.orientation.w = 1.0;
      // grasp_.pose.orientation.x = 0.0;
      // grasp_.pose.orientation.y = 0.0;
      // grasp_.pose.orientation.z = 0.0;

      // std::cout << "Grasp : " << grasp_ << std::endl;

      // send feedback to action client
      feedback_.grasp_candidates.emplace_back(grasp_);

      // q_val_ (probability of success), if there is more than one grasp
      // the cost = 1.0 - q_val_ to represent cost
      feedback_.costs.emplace_back(q_val_);

      server_->publishFeedback(feedback_);
      result_.grasp_state = "success";
      server_->setSucceeded(result_);
    }

    else{
      ROS_ERROR_NAMED(LOGNAME, "Failed to call gqcnn_grasp service");
      result_.grasp_state = "failed";
      server_->setAborted(result_);
    }
  }


private:
  ros::NodeHandle nh_;
  ros::ServiceClient gqcnn_client_;  // gqcnn service client
  ros::ServiceClient image_client_;  // image saving service client

  std::unique_ptr<actionlib::SimpleActionServer<moveit_task_constructor_msgs::SampleGraspPosesAction>>
      server_;                                                            // action server
  moveit_task_constructor_msgs::SampleGraspPosesFeedback feedback_;  // action feedback message
  moveit_task_constructor_msgs::SampleGraspPosesResult result_;      // action result message

  geometry_msgs::PoseStamped grasp_;  // best grasp
  double q_val_;                      // probability of success

  std::string goal_name_;           // action name
  std::string action_name_;         // action namespace
  std::string frame_id_;            // frame of point cloud/grasps

  std::string image_dir_;          // directory images saved
  std::string color_image_file_; // file path to color image
  std::string depth_image_file_; // file path to depth image

  bool load_images_;                // load images from file

  Eigen::Isometry3d trans_base_cam_;    // transformation from base link to camera link
  Eigen::Isometry3d transform_cam_opt_; // transformation from camera link to optical link
};
} // namespace moveit_task_constructor_dexnet



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
