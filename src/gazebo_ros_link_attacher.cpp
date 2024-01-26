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

#include <gazebo_ros_link_attacher/gazebo_ros_link_attacher.hpp>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosLinkAttacher)

// Constructor
GazeboRosLinkAttacher::GazeboRosLinkAttacher()
{
}

// Destructor
GazeboRosLinkAttacher::~GazeboRosLinkAttacher()
{
}

void GazeboRosLinkAttacher::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->node_ = gazebo_ros::Node::Get(_sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok()) {
    RCLCPP_FATAL_STREAM(
      node_->get_logger(),
      "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->world = _world;
  this->physics = this->world->Physics();
  this->attach_service_ = this->node_->create_service<gazebo_ros_link_attacher::srv::Attach>(
    "attach",
    std::bind(
      &GazeboRosLinkAttacher::attach_callback, this, std::placeholders::_1,
      std::placeholders::_2));
  RCLCPP_INFO_STREAM(node_->get_logger(), "Attach service created");
  this->detach_service_ = this->node_->create_service<gazebo_ros_link_attacher::srv::Attach>(
    "detach",
    std::bind(
      &GazeboRosLinkAttacher::detach_callback, this, std::placeholders::_1,
      std::placeholders::_2));
  RCLCPP_INFO_STREAM(node_->get_logger(), "Detach service created");
  RCLCPP_INFO(node_->get_logger(), "Link attacher node initialized.");
}

bool GazeboRosLinkAttacher::attach(
  std::string model1, std::string link1,
  std::string model2, std::string link2)
{
  // look for any previous instance of the joint first.
  // if we try to create a joint in between two links
  // more than once (even deleting any reference to the first one)
  // gazebo hangs/crashes
  fixedJoint fixed_joint;
  if (this->getJoint(model1, link1, model2, link2, fixed_joint)) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Joint already existed, reusing it.");
    fixed_joint.joint->Attach(fixed_joint.l1, fixed_joint.l2);
    return true;
  } else {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Creating new joint.");
  }
  fixed_joint.model1 = model1;
  fixed_joint.link1 = link1;
  fixed_joint.model2 = model2;
  fixed_joint.link2 = link2;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Getting BasePtr of " << model1);
  physics::BasePtr b1 = this->world->ModelByName(model1);

  if (b1 == NULL) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), model1 << " model was not found");
    return false;
  }
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Getting BasePtr of " << model2);
  physics::BasePtr b2 = this->world->ModelByName(model2);
  if (b2 == NULL) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), model2 << " model was not found");
    return false;
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Casting into ModelPtr");
  physics::ModelPtr m1(dynamic_cast<physics::Model *>(b1.get()));
  fixed_joint.m1 = m1;
  physics::ModelPtr m2(dynamic_cast<physics::Model *>(b2.get()));
  fixed_joint.m2 = m2;

  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "Getting link: '" << link1 << "' from model: '" << model1 << "'");
  physics::LinkPtr l1 = m1->GetLink(link1);
  if (l1 == NULL) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), link1 << " link was not found");
    return false;
  }
  if (l1->GetInertial() == NULL) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "link1 inertia is NULL!");
  } else {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      "link1 inertia is not NULL, for example, mass is: " << l1->GetInertial()->Mass());
  }
  fixed_joint.l1 = l1;
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "Getting link: '" << link2 << "' from model: '" << model2 << "'");
  physics::LinkPtr l2 = m2->GetLink(link2);
  if (l2 == NULL) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), link2 << " link was not found");
    return false;
  }
  if (l2->GetInertial() == NULL) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "link2 inertia is NULL!");
  } else {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      "link2 inertia is not NULL, for example, mass is: " << l2->GetInertial()->Mass());
  }
  fixed_joint.l2 = l2;

  RCLCPP_DEBUG_STREAM(
    node_->get_logger(),
    "Links are: " << l1->GetName() << " and " << l2->GetName());

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Creating revolute joint on model: '" << model1 << "'");
  fixed_joint.joint = this->physics->CreateJoint("revolute", m1);
  this->joints_.push_back(fixed_joint);

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Attach");
  fixed_joint.joint->Attach(l1, l2);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Loading links");
  fixed_joint.joint->Load(l1, l2, ignition::math::Pose3d());
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "SetModel");
  fixed_joint.joint->SetModel(m2);
  /*
    * If SetModel is not done we get:
    * ***** Internal Program Error - assertion (this->GetParentModel() != __null)
    failed in void gazebo::physics::Entity::PublishPose():
    /tmp/buildd/gazebo2-2.2.3/gazebo/physics/Entity.cc(225):
    An entity without a parent model should not happen

    * If SetModel is given the same model than CreateJoint given
    * Gazebo crashes with
    * ***** Internal Program Error - assertion (self->inertial != __null)
    failed in static void gazebo::physics::ODELink::MoveCallback(dBodyID):
    /tmp/buildd/gazebo2-2.2.3/gazebo/physics/ode/ODELink.cc(183): Inertial pointer is NULL
  */

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "SetHightstop");
  fixed_joint.joint->SetUpperLimit(0, 0);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "SetLowStop");
  fixed_joint.joint->SetLowerLimit(0, 0);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Init");
  fixed_joint.joint->Init();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Attach finished.");

  return true;
}

bool GazeboRosLinkAttacher::detach(
  std::string model1, std::string link1,
  std::string model2, std::string link2)
{
  // search for the instance of joint and do detach
  fixedJoint fixed_joint;
  if (this->getJoint(model1, link1, model2, link2, fixed_joint)) {
    fixed_joint.joint->Detach();
    return true;
  }

  return false;
}

bool GazeboRosLinkAttacher::getJoint(
  std::string model1, std::string link1,
  std::string model2, std::string link2,
  fixedJoint & joint)
{
  fixedJoint fixed_joint;
  for (std::vector<fixedJoint>::iterator it = this->joints_.begin(); it != this->joints_.end();
    ++it)
  {
    fixed_joint = *it;
    if ((fixed_joint.model1.compare(model1) == 0) && (fixed_joint.model2.compare(model2) == 0) &&
      (fixed_joint.link1.compare(link1) == 0) && (fixed_joint.link2.compare(link2) == 0))
    {
      joint = fixed_joint;
      return true;
    }
  }
  return false;
}

bool GazeboRosLinkAttacher::attach_callback(
  const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Request> req,
  std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Response> res)
{
  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "Received request to attach model: '"
      << req->model_name_1
      << "' using link: '" << req->link_name_1 << "' with model: '"
      << req->model_name_2 << "' using link: '" << req->link_name_2 <<
      "'"
  );
  if (!this->attach(
      req->model_name_1, req->link_name_1,
      req->model_name_2, req->link_name_2))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not make the attach.");
    res->ok = false;
  } else {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Attach was succesful");
    res->ok = true;
  }
  return true;
}

bool GazeboRosLinkAttacher::detach_callback(
  const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Request> req,
  std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Response> res)
{
  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "Received request to detach model: '"
      << req->model_name_1
      << "' using link: '" << req->link_name_1 << "' with model: '"
      << req->model_name_2 << "' using link: '" << req->link_name_2 <<
      "'"
  );
  if (!this->detach(
      req->model_name_1, req->link_name_1,
      req->model_name_2, req->link_name_2))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not make the detach.");
    res->ok = false;
  } else {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Detach was succesful");
    res->ok = true;
  }
  return true;
}

}  //  namespace gazebo
