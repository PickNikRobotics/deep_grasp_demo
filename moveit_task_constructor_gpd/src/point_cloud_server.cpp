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
   Desc:  Point cloud server node for saving point cloud data. Optionally, either
          the ground plane is removed and/or points outside the specified cartesian
          limits are removed. The resulting cloud is published.

  PARAMETERS:
    point_cloud_topic - point cloud topic to subscribe to
    cloud_dir - directory to save point clouds to
    remove_table - whether or not to segement table points
    cartesian_limits - whether or not to remove points outside limits
    xyz_lower_limits - lower (x,y,z) limits on points
    xyz_upper_limits - upper (x,y,z) limits on points
  PUBLISHES:
    filtered_cloud (sensor_msgs/PointCloud2) - Point cloud after table is removed
  SUBSCRIBES:
    point_cloud_topic (sensor_msgs/PointCloud2) - Point cloud from depth camera
  SERVICES:
    save_point_cloud (moveit_task_constructor_gpd/PointCloud) - specify file name to save point cloud
*/

// ROS
#include <ros/ros.h>

#include <moveit_task_constructor_gpd/cloud_server.h>

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("main", "Starting point_cloud_server");
  ros::init(argc, argv, "point_cloud_server");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_task_constructor_gpd::CloudServer cloud_server(nh);
  ros::waitForShutdown();

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  return 0;
}
