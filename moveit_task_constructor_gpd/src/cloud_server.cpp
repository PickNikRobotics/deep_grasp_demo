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

#include <moveit_task_constructor_gpd/cloud_utils.h>
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
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "point_cloud_topic", point_cloud_topic_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "cloud_dir", cloud_dir_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "remove_table", remove_table_);

  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "cartesian_limits", cartesian_limits_);
  if (cartesian_limits_)
  {
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "xyz_lower_limits", xyz_lower_limits_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "xyz_upper_limits", xyz_upper_limits_);
  }

  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void CloudServer::init()
{
  cloud_sub_ = nh_.subscribe(point_cloud_topic_, 1, &CloudServer::cloudCallback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1, true);
  saver_srv_ = nh_.advertiseService("save_point_cloud", &CloudServer::saveCallback, this);
}

void CloudServer::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (save_)
  {
    // convert from ROS msg to a point cloud
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    pcl::fromROSMsg(*msg.get(), *cloud.get());

    // segment out table
    if (remove_table_)
    {
      removeTable(cloud);
    }

    // remove points out of limits
    if (cartesian_limits_)
    {
      passThroughFilter(xyz_lower_limits_, xyz_upper_limits_, cloud);
    }

    // publish the cloud for visualization and debugging purposes
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud.get(), cloud_msg);
    cloud_pub_.publish(cloud_msg);

    if (!cloud->points.empty())
    {
      ROS_INFO_NAMED(LOGNAME, "Saving point cloud to file...");

      if (!pcl::io::savePCDFile(cloud_dir_ + file_name_, *cloud.get()))
      {
        ROS_INFO_NAMED(LOGNAME, "Point cloud saved");
      }
      else
      {
        ROS_ERROR_NAMED(LOGNAME, "Failed to save cloud");
      }
    }
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "Point cloud is empty");
    }

    save_ = false;
  }
}

bool CloudServer::saveCallback(moveit_task_constructor_gpd::PointCloud::Request& req,
                               moveit_task_constructor_gpd::PointCloud::Response&)
{
  ROS_INFO_NAMED(LOGNAME, "Saving point cloud service active");
  save_ = true;
  file_name_ = req.cloud_file;
  return true;
}
}  // namespace moveit_task_constructor_gpd
