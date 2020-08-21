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
   Desc:   Point cloud server for saving point cloud data
*/

// ROS
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <moveit_task_constructor_gpd/cloud_server.h>


namespace moveit_task_constructor_gpd
{
CloudServer::CloudServer(ros::NodeHandle& nh) : nh_(nh), save_(false)
{
  loadParameters();
  init();
}


void CloudServer::loadParameters()
{
  ros::NodeHandle pnh("~");
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "cloud_topic", cloud_topic_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "cloud_dir", cloud_dir_);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}


void CloudServer::init()
{
  cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &CloudServer::cloudCallback, this);
  saver_srv_ = nh_.advertiseService("save_point_cloud", &CloudServer::saveCallback, this);
}


void CloudServer::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if(save_){
    // convert from ROS msg to a point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg.get(), *cloud.get());
    removeTable(cloud);

    if (!cloud->points.empty()){
      ROS_INFO_NAMED(LOGNAME, "Saving point cloud to file...");

      if (!pcl::io::savePCDFile(cloud_dir_ + file_name_, *cloud.get())){
        ROS_INFO_NAMED(LOGNAME, "Point cloud saved");
      } else{
        ROS_ERROR_NAMED(LOGNAME, "Failed to save cloud");
      }
    } else{
      ROS_ERROR_NAMED(LOGNAME, "Point cloud is empty");
    }

    save_ = false;
  }
}


bool CloudServer::saveCallback(moveit_task_constructor_gpd::PointCloud::Request& req, moveit_task_constructor_gpd::PointCloud::Response&)
{
  ROS_INFO_NAMED(LOGNAME, "Saving point cloud service active");
  save_ = true;
  file_name_ = req.cloud_file;
  return true;
}


void CloudServer::removeTable(PointCloudRGB::Ptr cloud)
{
  // SAC segmentor without normals
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);

  // Max iterations and model tolerance
  segmentor.setMaxIterations(1000);
  segmentor.setDistanceThreshold (0.01);

  // Input cloud
  segmentor.setInputCloud(cloud);

  // Inliers representing points in the plane
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // Use a plane as the model for the segmentor
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  segmentor.segment(*inliers_plane, *coefficients_plane);

  if (inliers_plane->indices.size () == 0)
  {
    ROS_ERROR_NAMED(LOGNAME, "Could not estimate a planar model for the given dataset");
  }

  // Extract the inliers from the cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);

  // Remove plane inliers and extract the rest
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);

  if (cloud->points.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "Can't find objects");
  }
}
} // namespace moveit_task_constructor_gpd
