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

#pragma once

// ROS
#include <sensor_msgs/PointCloud2.h>

// C++
#include <string>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <moveit_task_constructor_gpd/PointCloud.h>

namespace moveit_task_constructor_gpd
{
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
constexpr char LOGNAME[] = "cloud_server";

class CloudServer
{
public:
  CloudServer(ros::NodeHandle& nh);

  void loadParameters();

  void init();

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  bool saveCallback(moveit_task_constructor_gpd::PointCloud::Request& req, moveit_task_constructor_gpd::PointCloud::Response&);

  void removeTable(PointCloudRGB::Ptr cloud);

private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::ServiceServer saver_srv_;

  std::string cloud_topic_;
  std::string cloud_dir_;
  std::string file_name_;

  bool save_;
};
} // namespace moveit_task_constructor_gpd
