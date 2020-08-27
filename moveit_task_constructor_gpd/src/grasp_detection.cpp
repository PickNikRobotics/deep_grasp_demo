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
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <pcl_conversions/pcl_conversions.h>

// Eigen
#include <Eigen/Dense>

#include <moveit_task_constructor_gpd/grasp_detection.h>
#include <moveit_task_constructor_gpd/cloud_utils.h>

namespace moveit_task_constructor_gpd
{
GraspDetection::GraspDetection(const ros::NodeHandle& nh) : nh_(nh), goal_active_(false)
{
  loadParameters();
  init();
}

void GraspDetection::loadParameters()
{
  ROS_INFO_NAMED(LOGNAME, "Loading grasp action server parameters");
  ros::NodeHandle pnh("~");

  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "load_cloud", load_cloud_);
  if (load_cloud_)
  {
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "path_to_pcd_file", path_to_pcd_file_);
  }
  else
  {
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "point_cloud_topic", point_cloud_topic_);
  }

  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "path_to_gpd_config", path_to_gpd_config_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "trans_cam_opt", transform_cam_opt_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "trans_base_cam", trans_base_cam_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "action_name", action_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "view_point", view_point_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "frame_id", frame_id_);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void GraspDetection::init()
{
  // action server
  server_.reset(new actionlib::SimpleActionServer<moveit_task_constructor_msgs::SampleGraspPosesAction>(
      nh_, action_name_, false));
  server_->registerGoalCallback(std::bind(&GraspDetection::goalCallback, this));
  server_->registerPreemptCallback(std::bind(&GraspDetection::preemptCallback, this));
  server_->start();

  // GPD point cloud camera, load cylinder from file
  // set camera view origin
  // assume cloud was taken using one camera
  if (load_cloud_)
  {
    Eigen::Matrix3Xd camera_view_point(3, 1);
    camera_view_point << view_point_.at(0), view_point_.at(1), view_point_.at(2);
    cloud_camera_.reset(new gpd::util::Cloud(path_to_pcd_file_, camera_view_point));
  }
  else
  {
    cloud_sub_ = nh_.subscribe(point_cloud_topic_, 1, &GraspDetection::cloudCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1, true);
  }

  // Grasp detector
  grasp_detector_.reset(new gpd::GraspDetector(path_to_gpd_config_));
}

void GraspDetection::goalCallback()
{
  goal_name_ = server_->acceptNewGoal()->action_name;
  ROS_INFO_NAMED(LOGNAME, "New goal accepted: %s", goal_name_.c_str());
  goal_active_ = true;

  // sample grasps now else need to wait to callback
  // use GPD to find the grasp candidates
  if (load_cloud_)
  {
    sampleGrasps();
  }
}

void GraspDetection::preemptCallback()
{
  ROS_INFO_NAMED(LOGNAME, "Preempted %s:", goal_name_.c_str());
  server_->setPreempted();
}

void GraspDetection::sampleGrasps()
{
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;  // detect grasp poses
  grasp_detector_->preprocessPointCloud(*cloud_camera_);      // preprocess the point cloud
  grasps = grasp_detector_->detectGrasps(*cloud_camera_);     // detect grasps in the point cloud

  // Use grasps with score > 0
  std::vector<unsigned int> grasp_id;
  for (unsigned int i = 0; i < grasps.size(); i++)
  {
    if (grasps.at(i)->getScore() > 0.0)
    {
      grasp_id.push_back(i);
    }
  }

  if (grasp_id.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "No grasp candidates found with a positive cost");
    result_.grasp_state = "failed";
    server_->setAborted(result_);
    return;
  }

  for (auto id : grasp_id)
  {
    // transform grasp from camera optical link into frame_id (panda_link0)
    const Eigen::Isometry3d transform_opt_grasp =
        Eigen::Translation3d(grasps.at(id)->getPosition()) * Eigen::Quaterniond(grasps.at(id)->getOrientation());

    const Eigen::Isometry3d transform_base_grasp = trans_base_cam_ * transform_cam_opt_ * transform_opt_grasp;
    const Eigen::Vector3d trans = transform_base_grasp.translation();
    const Eigen::Quaterniond rot(transform_base_grasp.rotation());

    // convert back to PoseStamped
    geometry_msgs::PoseStamped grasp_pose;
    grasp_pose.header.frame_id = frame_id_;
    grasp_pose.pose.position.x = trans.x();
    grasp_pose.pose.position.y = trans.y();
    grasp_pose.pose.position.z = trans.z();

    grasp_pose.pose.orientation.w = rot.w();
    grasp_pose.pose.orientation.x = rot.x();
    grasp_pose.pose.orientation.y = rot.y();
    grasp_pose.pose.orientation.z = rot.z();

    feedback_.grasp_candidates.emplace_back(grasp_pose);

    // Grasp is selected based on cost not score
    // Invert score to represent grasp with lowest cost
    feedback_.costs.emplace_back(static_cast<double>(1.0 / grasps.at(id)->getScore()));
  }

  server_->publishFeedback(feedback_);
  result_.grasp_state = "success";
  server_->setSucceeded(result_);
}

void GraspDetection::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (goal_active_)
  {
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    pcl::fromROSMsg(*msg.get(), *cloud.get());

    // Segementation works best with XYXRGB
    removeTable(cloud);

    // publish the cloud for visualization and debugging purposes
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud.get(), cloud_msg);
    cloud_pub_.publish(cloud_msg);

    // TODO: set alpha channel to 1
    // GPD required XYZRGBA
    PointCloudRGBA::Ptr grasp_cloud(new PointCloudRGBA);
    pcl::copyPointCloud(*cloud.get(), *grasp_cloud.get());

    // Construct the cloud camera
    Eigen::Matrix3Xd camera_view_point(3, 1);
    camera_view_point << view_point_.at(0), view_point_.at(1), view_point_.at(2);
    cloud_camera_.reset(new gpd::util::Cloud(grasp_cloud, 0, camera_view_point));

    // use GPD to find the grasp candidates
    sampleGrasps();
  }

  goal_active_ = false;
}
}  // namespace moveit_task_constructor_gpd
