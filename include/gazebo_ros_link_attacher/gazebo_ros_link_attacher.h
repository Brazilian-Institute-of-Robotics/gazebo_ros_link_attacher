// Copyright (c) 2024, SENAI Cimatec
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file gazebo_ros_link_attacher.h
 *
 * @brief Gazebo link attacher plugin class
 *
 * @date April 05, 2016
 * @authors Sammy Pfeiffer <sam.pfeiffer@pal-robotics.com>
 *          Claudia Ramos  <claudia.ramos@fieb.org.br>
 */

#ifndef GAZEBO_ROS_LINK_ATTACHER__GAZEBO_ROS_LINK_ATTACHER_H_
#define GAZEBO_ROS_LINK_ATTACHER__GAZEBO_ROS_LINK_ATTACHER_H_

#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros_link_attacher/srv/attach.hpp>
#include <ignition/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>

namespace gazebo
{
/// \brief Internal representation of a fixed joint
struct fixedJoint
{
  std::string model1;
  physics::ModelPtr m1;
  std::string link1;
  physics::LinkPtr l1;
  std::string model2;
  physics::ModelPtr m2;
  std::string link2;
  physics::LinkPtr l2;
  physics::JointPtr joint;
};

class GazeboRosLinkAttacher: public WorldPlugin
{
public:
  /// \brief Constructor
  GazeboRosLinkAttacher();

  /// \brief Destructor
  virtual ~GazeboRosLinkAttacher();

  /// \brief Load the controller
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  /// \brief Attach with a revolute joint
  bool attach(
    std::string model1, std::string link1,
    std::string model2, std::string link2);

  /// \brief Detach
  bool detach(
    std::string model1, std::string link1,
    std::string model2, std::string link2);

  bool getJoint(
    std::string model1, std::string link1, std::string model2, std::string link2,
    fixedJoint & joint);

private:
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service < gazebo_ros_link_attacher::srv::Attach > ::SharedPtr attach_service_;
  rclcpp::Service < gazebo_ros_link_attacher::srv::Attach > ::SharedPtr detach_service_;

  bool attach_callback(
    const std::shared_ptr < gazebo_ros_link_attacher::srv::Attach::Request > req,
    std::shared_ptr < gazebo_ros_link_attacher::srv::Attach::Response > res);
  bool detach_callback(
    const std::shared_ptr < gazebo_ros_link_attacher::srv::Attach::Request > req,
    std::shared_ptr < gazebo_ros_link_attacher::srv::Attach::Response > res);

  std::vector < fixedJoint > joints_;

  /// \brief The physics engine.
  physics::PhysicsEnginePtr physics;

  /// \brief Pointer to the world.
  physics::WorldPtr world;
};
}  // namespace gazebo
#endif  // GAZEBO_ROS_LINK_ATTACHER__GAZEBO_ROS_LINK_ATTACHER_H_
