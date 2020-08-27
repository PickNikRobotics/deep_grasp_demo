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
   Desc:   Image server for saving images
*/

#pragma once

// ROS
#include <sensor_msgs/Image.h>

// C++
#include <string>

#include <moveit_task_constructor_dexnet/Images.h>

namespace moveit_task_constructor_dexnet
{
constexpr char LOGNAME[] = "image_server";

/**
* @brief Provides a service for saving RGB and depth images
*/
class ImageServer
{
public:
  /**
  * @brief Constructor
  * @param nh - node handle
  */
  ImageServer(ros::NodeHandle& nh);

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
  * @brief RGB image callback
  * @param msg - GGB image
  */
  void colorCallback(const sensor_msgs::Image::ConstPtr& msg);

  /**
  * @brief Depth image callback
  * @param msg - Depth image
  */
  void depthCallback(const sensor_msgs::Image::ConstPtr& msg);

  /**
  * @brief Service callback for saving images
  * @param req - Service request contains the file name
  * @param res [out] - Service result true if image type is saved
  */
  bool saveCallback(moveit_task_constructor_dexnet::Images::Request& req,
                    moveit_task_constructor_dexnet::Images::Response& res);

  /**
  * @brief Saves image based on encoding and to specified file
  * @param msg - Image
  * @param image_name - File name of image
  * @details Images are saved as CV_8UC3 (BGR8) by OpenCV by default
  */
  bool saveImage(const sensor_msgs::Image::ConstPtr& msg, const std::string& image_name);

private:
  ros::NodeHandle nh_;             // node handle
  ros::Subscriber color_img_sub_;  // color image subscriber
  ros::Subscriber depth_img_sub_;  // depth image subscriber
  ros::ServiceServer saver_srv_;   // image saver service

  sensor_msgs::Image::ConstPtr color_img_;  // image to save
  sensor_msgs::Image::ConstPtr depth_img_;  // image to save

  std::string color_img_topic_;  // color image topic name
  std::string depth_img_topic_;  // depth image topic name
  std::string image_dir_;        // directory to save images
};
}  // namespace moveit_task_constructor_dexnet
