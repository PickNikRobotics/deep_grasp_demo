/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik LLC.
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
   Desc:   Image server
*/

#pragma once

// ROS
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// C++
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gqcnn_demo/Images.h>


namespace gqcnn_demo
{
constexpr char LOGNAME[] = "image_server";

class ImageServer
{
public:
  ImageServer(ros::NodeHandle& nh);

  void loadParameters();

  void init();

  void colorCallback(const sensor_msgs::Image::ConstPtr &msg);

  void depthCallback(const sensor_msgs::Image::ConstPtr &msg);

  bool saveCallback(gqcnn_demo::Images::Request& req, gqcnn_demo::Images::Response& res);

  void saveImage(const sensor_msgs::Image::ConstPtr &msg, const std::string &image_name);


private:
  ros::NodeHandle nh_;
  ros::Subscriber color_img_sub_;
  ros::Subscriber depth_img_sub_;
  ros::ServiceServer saver_srv_;

  std::string color_img_topic_;
  std::string depth_img_topic_;
  std::string image_dir_;

  std::string depth_file_;
  std::string color_file_;

  bool save_rbg_;
  bool save_depth_;
};

} // namespace gqcnn_demo
