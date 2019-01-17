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

#include <cstdlib>
#include <string>

#include <osrf_gear/ShippingBoxContents.h>

#include "ROSAriacShippingBoxPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ShippingBoxPlugin)

/////////////////////////////////////////////////
ShippingBoxPlugin::ShippingBoxPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
ShippingBoxPlugin::~ShippingBoxPlugin()
{
  this->updateConnection.reset();
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void ShippingBoxPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);

  if (_sdf->HasElement("toggle_visuals_at"))
  {
    sdf::ElementPtr toggleVisualsAtElem = _sdf->GetElement("toggle_visuals_at");
    this->toggleVisualsAtPose = true;
    this->toggleVisualsAt = toggleVisualsAtElem->Get<ignition::math::Vector3d>();
    std::string topicName = "/ariac/" + _model->GetName() + "_visual_toggle";
    this->toggleVisualsPub = this->node->Advertise<msgs::GzString>(topicName);
  }

  if (_sdf->HasElement("clear_models_at"))
  {
    sdf::ElementPtr clearModelsAtElem = _sdf->GetElement("clear_models_at");
    this->clearModelsAtPose = true;
    this->clearModelsAt = clearModelsAtElem->Get<ignition::math::Vector3d>();
  }

  if (_sdf->HasElement("trigger_animation_at"))
  {
    sdf::ElementPtr triggerAnimationAtElem = _sdf->GetElement("trigger_animation_at");
    this->triggerAnimationAtPose = true;
    this->triggerAnimationAt = triggerAnimationAtElem->Get<ignition::math::Vector3d>();
    sdf::ElementPtr endAnimationAtElem = _sdf->GetElement("end_animation_at");
    this->endAnimationAt = endAnimationAtElem->Get<ignition::math::Vector3d>();
  }

  if (_sdf->HasElement("nested_animation"))
  {
    this->nestedAnimation = _sdf->Get<bool>("nested_animation");
  }

  if (_sdf->HasElement("faulty_products"))
  {
    this->faultyProductNames.clear();
    sdf::ElementPtr faultyProductNamesElem = _sdf->GetElement("faulty_products");
    if (faultyProductNamesElem->HasElement("name"))
    {
      sdf::ElementPtr faultyProductElem = faultyProductNamesElem->GetElement("name");
      while (faultyProductElem)
      {
        std::string faultyProductName = faultyProductElem->Get<std::string>();

        ROS_DEBUG_STREAM("Ignoring product: " << faultyProductName);
        this->faultyProductNames.push_back(faultyProductName);
        faultyProductElem = faultyProductElem->GetNextElement("name");
      }
    }
  }

  if (this->updateRate > 0)
    gzdbg << "ShippingBoxPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "ShippingBoxPlugin running at the default update rate\n";

  this->shippingBoxID = this->parentLink->GetScopedName();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode = new ros::NodeHandle("");
  this->currentShipmentPub = this->rosNode->advertise<osrf_gear::ShippingBoxContents>(
    "/ariac/shipping_boxes", 1000, boost::bind(&ShippingBoxPlugin::OnSubscriberConnect, this, _1));
  this->publishingEnabled = true;

  // ROS service for clearing the shipping box
  std::string clearServiceName = "clear";
  if (_sdf->HasElement("clear_shipping_box_service_name"))
    clearServiceName = _sdf->Get<std::string>("clear_shipping_box_service_name");

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (!compRunning)
  {
    this->clearShippingBoxServer =
      this->rosNode->advertiseService(clearServiceName, &ShippingBoxPlugin::HandleClearService, this);
  }

  // Initialize Gazebo transport
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();

  // Gazebo subscription for the lock shipping boxes topic
  std::string lockModelsServiceName = "lock_models";
  if (_sdf->HasElement("lock_models_service_name"))
    lockModelsServiceName = _sdf->Get<std::string>("lock_models_service_name");
  this->lockModelsSub = this->gzNode->Subscribe(
    lockModelsServiceName, &ShippingBoxPlugin::HandleLockModelsRequest, this);

  double speedFactor = 1.0;
  this->rampAnimation.reset(
    new gazebo::common::PoseAnimation(this->shippingBoxID, 3/speedFactor, false));

  ignition::math::Vector3d start_position(this->triggerAnimationAt.X(), this->triggerAnimationAt.Y(), this->triggerAnimationAt.Z());
  ignition::math::Vector3d end_position(this->endAnimationAt.X(), this->endAnimationAt.Y(), this->endAnimationAt.Z());

  gazebo::common::PoseKeyFrame *key = this->rampAnimation->CreateKeyFrame(1.0/speedFactor);
  key->Translation(end_position + ignition::math::Vector3d(0, 0.3, 0.2));
  key->Rotation(ignition::math::Quaterniond(0.18, 0, 0));

  key = this->rampAnimation->CreateKeyFrame(2.0/speedFactor);
  key->Translation(end_position + ignition::math::Vector3d(0, 0.1, 0.18));
  key->Rotation(ignition::math::Quaterniond(0.07, 0, 0));

  key = this->rampAnimation->CreateKeyFrame(3.0/speedFactor);
  key->Translation(end_position);
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

}

/////////////////////////////////////////////////
void ShippingBoxPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
  {
    return;
  }

  if (this->toggleVisualsAtPose && this->model->WorldPose().Pos().Distance(this->toggleVisualsAt) < 0.3)
  {
    gzdbg << "Toggling visuals: " << this->model->GetName() << std::endl;
    gazebo::msgs::GzString toggleMsg;
    toggleMsg.set_data("toggle");
    this->toggleVisualsPub->Publish(toggleMsg);
    this->toggleVisualsAtPose = false;
  }

  if (this->clearModelsAtPose && this->model->WorldPose().Pos().Distance(this->clearModelsAt) < 0.3)
  {
    gzdbg << "Clearing models: " << this->model->GetName() << std::endl;
    this->ClearContactingModels();
    this->clearModelsAtPose = false;
    // Stop publishing so that the previous state is the last one known to the scorer.
    this->publishingEnabled = false;
  }

  if (this->triggerAnimationAtPose && this->model->WorldPose().Pos().Distance(this->triggerAnimationAt) < 0.3)
  {
    gzdbg << "Triggering ramp animation: " << this->model->GetName() << std::endl;

    // Make the current pose the first key frame of the animation (appears smoother).
    gazebo::common::PoseKeyFrame *key = this->rampAnimation->CreateKeyFrame(0.0);
    ignition::math::Vector3d pos = this->model->WorldPose().Pos();
    ignition::math::Quaterniond ori = this->model->WorldPose().Rot();
    key->Translation(ignition::math::Vector3d(pos.X(), pos.Y(), pos.Z()));
    key->Rotation(ignition::math::Quaterniond(ori.W(), ori.X(), ori.Y(), ori.Z()));

    // Trigger the animation.
    this->rampAnimation->SetTime(0);
    this->model->SetAnimation(this->rampAnimation);
    this->triggerAnimationAtPose = false;
  }

  if (!this->newMsg)
  {
    return;
  }

  auto prevNumberContactingModels = this->contactingModels.size();
  this->CalculateContactingModels();
  if (prevNumberContactingModels != this->contactingModels.size()) {
    ROS_DEBUG_STREAM(this->parentLink->GetScopedName() << ": number of contacting models: "
      << this->contactingModels.size());
  }
  this->ProcessContactingModels();
  if (this->publishingEnabled)
  {
    this->PublishShipmentMsg();
  }
}

/////////////////////////////////////////////////
void ShippingBoxPlugin::ProcessContactingModels()
{
  // Make sure that models fixed to the shipping box are included in the contacting models,
  // even if they aren't contacting the shipping box anymore.
  for (auto fixedJoint : this->fixedJoints)
  {
    auto link = fixedJoint->GetChild();
    this->contactingLinks.insert(link);
    this->contactingModels.insert(link->GetParentModel());
  }
  this->currentShipment.products.clear();
  auto shippingBoxPose = this->parentLink->WorldPose();
  for (auto model : this->contactingModels) {
    if (model) {
      model->SetAutoDisable(false);
      ariac::Product product;

      // Determine the product type
      product.type = ariac::DetermineModelType(model->GetName());

      // Determine if the product is faulty
      auto modelName = ariac::TrimNamespace(model->GetName());
      auto it = std::find(this->faultyProductNames.begin(), this->faultyProductNames.end(), modelName);
      product.isFaulty = it != this->faultyProductNames.end();

      // Determine the pose of the product in the frame of the shipping box
      ignition::math::Pose3d productPose = model->WorldPose();
      ignition::math::Matrix4d transMat(shippingBoxPose);
      ignition::math::Matrix4d productPoseMat(productPose);
      product.pose = (transMat.Inverse() * productPoseMat).Pose();
      product.pose.Rot().Normalize();

      this->currentShipment.products.push_back(product);
    }
  }
}

/////////////////////////////////////////////////
void ShippingBoxPlugin::OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub)
{
  auto subscriberName = pub.getSubscriberName();
  gzdbg << this->shippingBoxID << ": New subscription from node: " << subscriberName << std::endl;

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning && subscriberName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition is running so subscribing to this topic is not permitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    // Disable publishing of shipment messages.
    // This will break the scoring but ensure competitors can't cheat.
    this->publishingEnabled = false;
  }
}

/////////////////////////////////////////////////
void ShippingBoxPlugin::PublishShipmentMsg()
{
  // Publish current shipment
  osrf_gear::ShippingBoxContents shippingBoxMsg;
  shippingBoxMsg.shipping_box = this->shippingBoxID;
  for (const auto &obj : this->currentShipment.products)
  {
    osrf_gear::DetectedProduct msgObj;
    msgObj.type = obj.type;
    msgObj.is_faulty = obj.isFaulty;
    msgObj.pose.position.x = obj.pose.Pos().X();
    msgObj.pose.position.y = obj.pose.Pos().Y();
    msgObj.pose.position.z = obj.pose.Pos().Z();
    msgObj.pose.orientation.x = obj.pose.Rot().X();
    msgObj.pose.orientation.y = obj.pose.Rot().Y();
    msgObj.pose.orientation.z = obj.pose.Rot().Z();
    msgObj.pose.orientation.w = obj.pose.Rot().W();

    // Add the product to the shipment.
    shippingBoxMsg.products.push_back(msgObj);
  }
  this->currentShipmentPub.publish(shippingBoxMsg);
}

/////////////////////////////////////////////////
void ShippingBoxPlugin::UnlockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;
  for (auto fixedJoint : this->fixedJoints)
  {
    fixedJoint->Detach();
  }
  this->fixedJoints.clear();
}

/////////////////////////////////////////////////
void ShippingBoxPlugin::LockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;
  gzdbg << "Number of models in contact with the shipping box: " << this->contactingModels.size() << std::endl;
  for (auto model : this->contactingModels)
  {
  // Create the joint that will attach the models
  fixedJoint = this->world->Physics()->CreateJoint(
        "fixed", this->model);
  auto jointName = this->model->GetName() + "_" + model->GetName() + "__joint__";
  gzdbg << "Creating fixed joint: " << jointName << std::endl;
  fixedJoint->SetName(jointName);

  auto modelName = model->GetName();
  auto linkName = modelName + "::link";
  auto link = model->GetLink(linkName);
  if (link == NULL)
  {
    // If the model was inserted into the world using the "population" SDF tag,
    // the link will have an additional namespace of the model type.
    linkName = modelName + "::" + ariac::DetermineModelType(modelName) + "::link";
    link = model->GetLink(linkName);
    if (link == NULL)
    {
      gzwarn << "Couldn't find link to make joint with: " << linkName;
      continue;
    }
  }
  if (this->nestedAnimation)
  {
    model->SetGravityMode(false);
    link->SetGravityMode(false);

    // Lift the product slightly because it will fall through the shipping box if the shipping box is animated
    model->SetWorldPose(model->WorldPose() + ignition::math::Pose3d(0,0,0.01,0,0,0));
  }

  fixedJoint->Load(link, this->parentLink, ignition::math::Pose3d());
  fixedJoint->Attach(this->parentLink, link);
  fixedJoint->Init();
  this->fixedJoints.push_back(fixedJoint);
  model->SetAutoDisable(true);
  }
}

/////////////////////////////////////////////////
void ShippingBoxPlugin::HandleLockModelsRequest(ConstGzStringPtr &_msg)
{
  gzdbg << this->shippingBoxID << ": Handle clear shipping box service called.\n";
  (void)_msg;
  this->LockContactingModels();
}

/////////////////////////////////////////////////
bool ShippingBoxPlugin::HandleClearService(
  ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response>& event)
{
  std_srvs::Trigger::Response& res = event.getResponse();

  const std::string& callerName = event.getCallerName();
  gzdbg << this->shippingBoxID << ": Handle clear shipping box service called by: " << callerName << std::endl;

  this->UnlockContactingModels();
  this->ClearContactingModels();
  res.success = true;
  return true;
}
