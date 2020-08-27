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

/* Author: Henning Kayser, Simon Goldstein, Boston Cleek
   Desc:   A demo to show MoveIt Task Constructor using a deep learning based
           grasp generator
*/

// ROS
#include <ros/ros.h>

// MTC demo implementation
#include <deep_grasp_task/deep_pick_place_task.h>

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>

#include <geometric_shapes/shape_operations.h>
#include <moveit_task_constructor_msgs/SampleGraspPosesAction.h>
#include <actionlib/client/simple_action_client.h>

constexpr char LOGNAME[] = "deep_grasp_demo";

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object)
{
  if (!psi.applyCollisionObject(object))
    throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::CollisionObject createTable()
{
  ros::NodeHandle pnh("~");
  std::string table_name, table_reference_frame;
  std::vector<double> table_dimensions;
  geometry_msgs::Pose pose;
  std::size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_name", table_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_dimensions", table_dimensions);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

  moveit_msgs::CollisionObject object;
  object.id = table_name;
  object.header.frame_id = table_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions = table_dimensions;
  pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createObject()
{
  ros::NodeHandle pnh("~");
  std::string object_name, object_reference_frame;
  std::vector<double> object_dimensions;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = object_dimensions;
  pose.position.z += 0.5 * object_dimensions[0];
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createCamera()
{
  ros::NodeHandle pnh("~");
  std::string camera_name, camera_reference_frame, camera_mesh_file;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_name", camera_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_mesh_file", camera_mesh_file);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_reference_frame", camera_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  shapes::Mesh* obj_mesh = shapes::createMeshFromResource(camera_mesh_file);

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(obj_mesh, mesh_msg);
  shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  moveit_msgs::CollisionObject object;
  object.id = camera_name;
  object.header.frame_id = camera_reference_frame;
  object.meshes.emplace_back(mesh);
  object.mesh_poses.emplace_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createObjectMesh()
{
  ros::NodeHandle pnh("~");
  std::string object_name, object_reference_frame, object_mesh_file;
  std::vector<double> object_dimensions;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_mesh_file", object_mesh_file);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  shapes::Mesh* obj_mesh = shapes::createMeshFromResource(object_mesh_file);

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(obj_mesh, mesh_msg);
  shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.meshes.emplace_back(mesh);
  object.mesh_poses.emplace_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  // moveit_msgs::CollisionObject object;
  // object.id = object_name;
  // object.header.frame_id = object_reference_frame;
  // object.primitives.resize(1);
  // object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  // object.primitives[0].dimensions = object_dimensions;
  // pose.position.z += 0.5 * object_dimensions[0];
  // object.primitive_poses.push_back(pose);
  // object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

int main(int argc, char** argv)
{
  ROS_INFO_NAMED(LOGNAME, "Init deep_grasp_demo");
  ros::init(argc, argv, "deep_grasp_demo");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Wait for ApplyPlanningScene service
  ros::Duration(1.0).sleep();

  // Add table and object to planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  ros::NodeHandle pnh("~");
  if (pnh.param("spawn_table", true))
  {
    spawnObject(psi, createTable());
  }

  // Add camera to planning scene
  if (pnh.param("spawn_camera", true))
  {
    spawnObject(psi, createCamera());
  }

  // Add object to planning scene either as mesh or geometric primitive
  if (pnh.param("spawn_mesh", true))
  {
    spawnObject(psi, createObjectMesh());
  }
  else
  {
    spawnObject(psi, createObject());
  }

  // Construct and run task
  deep_grasp_task::DeepPickPlaceTask deep_pick_place_task("deep_pick_place_task", nh);
  deep_pick_place_task.loadParameters();
  deep_pick_place_task.init();

  if (deep_pick_place_task.plan())
  {
    ROS_INFO_NAMED(LOGNAME, "Planning succeded");
    if (pnh.param("execute", false))
    {
      deep_pick_place_task.execute();
      ROS_INFO_NAMED(LOGNAME, "Execution complete");
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Execution disabled");
    }
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }

  // Keep introspection alive
  ros::waitForShutdown();
  return 0;
}
