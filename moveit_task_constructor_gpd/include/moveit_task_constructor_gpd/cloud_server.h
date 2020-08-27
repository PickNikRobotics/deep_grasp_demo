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
#include <vector>

#include <moveit_task_constructor_gpd/PointCloud.h>

namespace moveit_task_constructor_gpd
{
constexpr char LOGNAME[] = "cloud_server";

/**
* @brief Provides a service for saving and process a point cloud
* @details When the service is called the point cloud received through
*          the point cloud topic is saved. Optionally, either the ground plane
*          is removed and/or points outside the specified cartesian limits
*          are removed. The resulting cloud is published.
*/
class CloudServer
{
public:
  /**
  * @brief Constructor
  * @param nh - node handle
  */
  CloudServer(ros::NodeHandle& nh);

private:
  /**
  * @brief Loads parameters
  */
  void loadParameters();

  /**
  * @brief Initialize ROS communication
  */
  void init();

  /**
  * @brief Point cloud call back
  * @param msg - point cloud message
  * @details Optionally, either the ground plane
  *          is removed and/or points outside the specified cartesian limits
  *          are removed. The resulting cloud is published.
  */
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  /**
  * @brief Service callback for saving a point cloud
  * @param req - Service request contains the file name
  * @return true when called
  * @details The response is empty
  */
  bool saveCallback(moveit_task_constructor_gpd::PointCloud::Request& req,
                    moveit_task_constructor_gpd::PointCloud::Response&);

private:
  ros::NodeHandle nh_;            // node handle
  ros::Subscriber cloud_sub_;     // point cloud subscriber
  ros::Publisher cloud_pub_;      // publishes the point cloud saved
  ros::ServiceServer saver_srv_;  // service for saving point clouds

  std::string point_cloud_topic_;  // point cloud topic
  std::string cloud_dir_;          // directory to save
  std::string file_name_;          // file name to save as

  std::vector<double> xyz_lower_limits_;  // lower limits on point cloud
  std::vector<double> xyz_upper_limits_;  // upper limits on point cloud

  bool save_;              // save service activated
  bool remove_table_;      // specify if to remove table points
  bool cartesian_limits_;  // specify if to remove points outside limits
};
}  // namespace moveit_task_constructor_gpd
