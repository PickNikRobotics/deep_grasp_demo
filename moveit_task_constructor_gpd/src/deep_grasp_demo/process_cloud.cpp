/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
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

 /* Author:  Boston Cleek
    Desc:  Pre-process a point cloud before grasp detection
 */


// PCL
#include <pcl/common/common.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <deep_grasp_demo/process_cloud.h>

namespace deep_grasp_demo
{
void removeTable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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
    ROS_ERROR("Could not estimate a planar model for the given dataset");
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
    ROS_ERROR("Can't find objects");
  }
}


void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_cloud)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(filter_cloud);

  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min, x_max);
  pass.filter(*filter_cloud);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min, y_max);
  pass.filter(*filter_cloud);

  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.filter(*filter_cloud);
}
} // namespace deep_grasp_demo
