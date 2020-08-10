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

#include <gqcnn_demo/image_server.h>

namespace gqcnn_demo
{

ImageServer::ImageServer(ros::NodeHandle& nh) : nh_(nh), save_rbg_(false), save_depth_(false)
{
  loadParameters();
  init();
}


void ImageServer::loadParameters()
{
  ros::NodeHandle pnh("~");
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "color_img_topic", color_img_topic_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "depth_img_topic", depth_img_topic_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "image_dir", image_dir_);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}


void ImageServer::init()
{
  color_img_sub_ = nh_.subscribe(color_img_topic_, 1, &ImageServer::colorCallback, this);
  depth_img_sub_ = nh_.subscribe(depth_img_topic_, 1, &ImageServer::depthCallback, this);
  saver_srv_ = nh_.advertiseService("save_images", &ImageServer::saveCallback, this);
}


void ImageServer::colorCallback(const sensor_msgs::Image::ConstPtr &msg)
{
  if (save_rbg_){
    saveImage(msg, "object_rgb.png");
    save_rbg_ = false;
  }
}


void ImageServer::depthCallback(const sensor_msgs::Image::ConstPtr &msg)
{
  if (save_depth_){
    saveImage(msg, "object_depth.png");
    save_depth_ = false;
  }
}


bool ImageServer::saveCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO_NAMED(LOGNAME, "Saving image service active");
  save_rbg_ = true;
  save_depth_ = true;
  return true;
}


void ImageServer::saveImage(const sensor_msgs::Image::ConstPtr &msg, const std::string &image_name)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

  // imwrite() saves image as BGR
  cv::Mat img_converted;

  if (msg->encoding == "rgb8"){
    cv_ptr->image.convertTo(img_converted, CV_8UC3); // convert to BGR
  }

  else if (msg->encoding == "32FC1"){
    cv_ptr->image.convertTo(img_converted, CV_8UC3, 255.0); // conver to BGR and scale
  }

  else{
    ROS_ERROR_NAMED(LOGNAME, "Image encoding not recognized (encoding): %s", msg->encoding.c_str());
  }


  if (cv::imwrite(image_dir_ + image_name, img_converted)){
    ROS_INFO_NAMED(LOGNAME, "Saving image %s (encoding): %s ", image_name.c_str(), msg->encoding.c_str());
  }
  else {
    ROS_WARN_NAMED(LOGNAME, "Image %s not saved", image_name.c_str());
  }
}
} // namespace gqcnn_demo
