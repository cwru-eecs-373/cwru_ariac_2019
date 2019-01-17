/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <limits>
#include <string>
#include <gazebo/transport/Node.hh>

#include "ObjectDisposalPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ObjectDisposalPlugin)

static const std::string SHIPPING_BOX_MODEL_NAME = "shipping_box";
static const int NUM_SHIPPING_BOXES = 10;

/////////////////////////////////////////////////
ObjectDisposalPlugin::ObjectDisposalPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
ObjectDisposalPlugin::~ObjectDisposalPlugin()
{
  this->updateConnection.reset();
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  if (this->updateRate > 0)
    gzdbg << "ObjectDisposalPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "ObjectDisposalPlugin running at the default update rate\n";

  this->centerOfGravityCheck = false;
  if (_sdf->HasElement("center_of_gravity_check"))
  {
    this->centerOfGravityCheck = _sdf->Get<bool>("center_of_gravity_check");
  }

  std::string activation_topic = "/ariac/" +  this->model->GetName() + "/activate";
  if (_sdf->HasElement("activation_topic"))
  {
    activation_topic = _sdf->Get<std::string>("activation_topic");
  }
  this->activationSub = this->node->Subscribe(
    activation_topic,
    &ObjectDisposalPlugin::OnActivation, this);

  if (!_sdf->HasElement("disposal_pose"))
  {
    gzerr << "ObjectDisposalPlugin: Unable to find <disposal_pose> element\n";
    return;
  }
  this->disposalPose = _sdf->Get<ignition::math::Pose3d>("disposal_pose");

  std::string currentBoxTopic = "/ariac/" +  this->model->GetName() + "/contacting_box";
  if (_sdf->HasElement("contacting_box_topic"))
  {
    currentBoxTopic = _sdf->Get<std::string>("contacting_box_topic");
  }
  this->currentBoxPub = this->node->Advertise<msgs::GzString>(currentBoxTopic);
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
    return;

  this->CalculateContactingModels();
  this->ActOnContactingModels();
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::ActOnContactingModels()
{
  // Publish if there's a detected box.
  // Don't publish all detected models because products will be detected too.
  gazebo::msgs::GzString currentBoxMsg;
  currentBoxMsg.set_data("");
  for (auto model : this->contactingModels)
  {
    if (!(model && model->GetName().compare(0, SHIPPING_BOX_MODEL_NAME.length(), SHIPPING_BOX_MODEL_NAME) == 0))
    {
      continue;
    }
    currentBoxMsg.set_data(model->GetName());
  }
  this->currentBoxPub->Publish(currentBoxMsg);

  if (!this->active)
    return;

  // Only remove models if their center of gravity is "above" the link
  // TODO: make more general than just z axis
  auto linkBox = this->parentLink->BoundingBox();
  auto linkBoxMax = linkBox.Max();
  auto linkBoxMin = linkBox.Min();
  linkBoxMin.Z() = std::numeric_limits<double>::lowest();
  linkBoxMax.Z() = std::numeric_limits<double>::max();
  auto disposalBox = ignition::math::Box(linkBoxMin, linkBoxMax);

  for (auto model : this->contactingModels) {
    // Only remove boxes because the contacting models will have already been locked
    if (!(model && model->GetName().compare(0, SHIPPING_BOX_MODEL_NAME.length(), SHIPPING_BOX_MODEL_NAME) == 0))
    {
      continue;
    }

    if (this->centerOfGravityCheck)
    {
      // Calculate the center of gravity of the model
      ignition::math::Vector3d modelCog = ignition::math::Vector3d::Zero;
      double modelMass = 0.0;
      for (auto modelLink : model->GetLinks())
      {
        double linkMass = modelLink->GetInertial()->Mass();
        modelCog += modelLink->WorldCoGPose().Pos() * linkMass;
        modelMass += linkMass;
      }
      if (modelMass > 0.0)
      {
        modelCog /= modelMass;
      }
      if (!disposalBox.Contains(modelCog))
      {
        continue;
      }
    }
    // Only update the position, not the orientation.
    ignition::math::Pose3d modelDisposalPose = model->WorldPose();
    modelDisposalPose.Pos() = this->disposalPose.Pos() + ignition::math::Vector3d(0, 1.25 * this->numRemovedModels++, 0);

    gzdbg << "[" << this->model->GetName() << "] Removing model: " << model->GetName() << "\n";
    model->SetAutoDisable(true);
    model->SetWorldPose(modelDisposalPose);
  }
  if (this->activateOnce)
  {
    this->activateOnce = false;
    this->active = false;
  }
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::OnActivation(ConstGzStringPtr &_msg)
{
  if (_msg->data() == "activate_once")
  {
    this->active = true;
    this->activateOnce = true;
  }
  else if (_msg->data() == "activate")
  {
    this->active = true;
  }
  else if (_msg->data() == "deactivate")
  {
    this->active = false;
  }
  else
  {
    gzerr << "Unknown activation command [" << _msg->data() << "]" << std::endl;
  }
}
