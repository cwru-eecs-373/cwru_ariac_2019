/*
 * Copyright 2016 Open Source Robotics Foundation
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
#include "ROSDronePlugin.hh"

#include <gazebo/common/common.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <osrf_gear/SubmitShipment.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <string>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSDronePlugin class.
  struct ROSDronePluginPrivate
  {
    /// \brief Name of the drone
    public: std::string droneName;

    /// \brief World pointer
    public: physics::WorldPtr world;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr updateConnection;

    /// \brief for setting ROS name space
    public: std::string robotNamespace;

    /// \brief ros node handle
    public: ros::NodeHandle *rosnode;

    /// \brief Receives service calls for controlling the drone
    public: ros::ServiceServer rosService;

    /// \brief Client for submitting shipping boxes for inspection
    public: ros::ServiceClient rosSubmitShipmentClient;

    /// \brief Transportation node.
    public: transport::NodePtr gzNode;

    /// \brief Gazebo publisher for toggling box visual visibility.
    public: transport::PublisherPtr toggleBoxVisualPub;

    /// \brief Gazebo publisher for removing boxes.
    public: transport::PublisherPtr clearBoxesPub;

    /// \brief Gazebo subscriber for boxes waiting to be collected.
    public: transport::SubscriberPtr waitingBoxSub;

    /// \brief Robot animation for collecting the shipment
    public: gazebo::common::PoseAnimationPtr collectAnimation;

    /// \brief Robot animation for the drone returning to its base
    public: gazebo::common::PoseAnimationPtr returnAnimation;

    /// \brief The pose from which models should be collected.
    public: ignition::math::Vector3d collectionPoint;

    /// \brief Pointer to the model
    public: gazebo::physics::ModelPtr model;

    /// \brief Type of shipment assigned to the drone
    public: std::string shipmentType;

    /// \brief The state of the drone
    public: std::string currentState;

    /// \brief The time the last delivery was triggered
    public: common::Time deliveryTriggerTime;

    /// \brief Name of the shipping box waiting to be collected ("" if none)
    public: std::string waitingBoxName;

    /// \brief Flag for triggering delivery from the service callback
    public: bool deliveryTriggered = false;

    /// \brief Evaluation result of the shipment (negative means invalid)
    public: int inspectionResult = -1;

    /// \brief Publishes the drone state.
    public: ros::Publisher statePub;
  };
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSDronePlugin);

/////////////////////////////////////////////////
ROSDronePlugin::ROSDronePlugin()
  : dataPtr(new ROSDronePluginPrivate)
{
}

/////////////////////////////////////////////////
ROSDronePlugin::~ROSDronePlugin()
{
  this->dataPtr->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSDronePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  std::string index = "";

  if (_sdf->HasElement("index"))
  {
    index = _sdf->Get<std::string>("index");
  }

  this->dataPtr->world = _parent->GetWorld();

  // load parameters
  this->dataPtr->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->dataPtr->robotNamespace = _sdf->GetElement(
        "robotNamespace")->Get<std::string>() + "/";
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->dataPtr->droneName = std::string("drone") + index;

  std::string droneControlTopic = "/ariac/" + this->dataPtr->droneName;
  ROS_DEBUG_STREAM("Using drone control service topic: " << droneControlTopic);

  std::string submitShipmentTopic = "submit_shipment";
  if (_sdf->HasElement("submit_shipment_service_name"))
    submitShipmentTopic = _sdf->Get<std::string>("submit_shipment_service_name");
  ROS_DEBUG_STREAM("Using submit shipment service topic: " << submitShipmentTopic);

  std::string waitingBoxTopic = "waiting_shipping_box";
  if (_sdf->HasElement("waiting_box_topic"))
  {
    waitingBoxTopic = _sdf->Get<std::string>("waiting_box_topic");
  }
  this->dataPtr->collectionPoint = _sdf->Get<ignition::math::Vector3d>("collection_point");

  this->dataPtr->rosnode = new ros::NodeHandle(this->dataPtr->robotNamespace);

  // Initialize Gazebo transport
  this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
  this->dataPtr->gzNode->Init();
  this->dataPtr->toggleBoxVisualPub =
    this->dataPtr->gzNode->Advertise<msgs::GzString>("~/drone_box_visual_toggle");
  this->dataPtr->clearBoxesPub =
    this->dataPtr->gzNode->Advertise<msgs::GzString>("/ariac/drone_collection_zone/activate_deletion");
  this->dataPtr->waitingBoxSub = this->dataPtr->gzNode->Subscribe(
    waitingBoxTopic, &ROSDronePlugin::OnWaitingBox, this);

  double speedFactor = 0.8;
  this->dataPtr->collectAnimation.reset(
    new gazebo::common::PoseAnimation(this->dataPtr->droneName, 4/speedFactor, false));

  ignition::math::Vector3d off_screen_position1(-1.4, -9.4, 4.3);
  ignition::math::Vector3d off_screen_position2(6.4, -12.4, 4.3);
  ignition::math::Vector3d lower_position = this->dataPtr->collectionPoint;
  ignition::math::Vector3d hover_position = lower_position + ignition::math::Vector3d(0.1, -0.2, 0.4);

  gazebo::common::PoseKeyFrame *key = this->dataPtr->collectAnimation->CreateKeyFrame(0);
  key->Translation(off_screen_position1);
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = this->dataPtr->collectAnimation->CreateKeyFrame(3.5/speedFactor);
  key->Translation(hover_position);
  key->Rotation(ignition::math::Quaterniond(0, 0, 1.3));


  key = this->dataPtr->collectAnimation->CreateKeyFrame(4/speedFactor);
  key->Translation(lower_position);
  key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

  this->dataPtr->returnAnimation.reset(
    new gazebo::common::PoseAnimation(this->dataPtr->droneName, 4/speedFactor, false));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(0);
  key->Translation(lower_position);
  key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(0.6/speedFactor);
  key->Translation(hover_position);
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(4.0/speedFactor);
  key->Translation(off_screen_position2);
  key->Rotation(ignition::math::Quaterniond(0, 0, -1.2));

  this->dataPtr->model = _parent;

  this->dataPtr->rosService = this->dataPtr->rosnode->advertiseService(droneControlTopic,
      &ROSDronePlugin::OnCommand, this);

  // Client for submitting shipping boxes for inspection.
  this->dataPtr->rosSubmitShipmentClient =
    this->dataPtr->rosnode->serviceClient<osrf_gear::SubmitShipment>(submitShipmentTopic);

  // Publisher for the status of the drone.
  std::string stateTopic = "/ariac/" + this->dataPtr->droneName + "/state";
  this->dataPtr->statePub = this->dataPtr->rosnode->advertise<
    std_msgs::String>(stateTopic, 1000);

  this->dataPtr->currentState = "ready_to_collect";

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ROSDronePlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void ROSDronePlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  auto currentSimTime = this->dataPtr->world->SimTime();
  if (this->dataPtr->currentState == "ready_to_collect")
  {
    if (this->dataPtr->deliveryTriggered)
    {
      this->dataPtr->deliveryTriggerTime = currentSimTime;
      // Make a service call to submit the shipment for inspection.
      // Do this before animating in case the assigned
      // goal changes as the drone is moving.
      if (!this->dataPtr->rosSubmitShipmentClient.exists())
      {
        this->dataPtr->rosSubmitShipmentClient.waitForExistence();
      }
      osrf_gear::SubmitShipment submit_srv;
      submit_srv.request.shipping_box_id = this->dataPtr->waitingBoxName + "::box_base";
      submit_srv.request.shipment_type = this->dataPtr->shipmentType;
      this->dataPtr->rosSubmitShipmentClient.call(submit_srv);
      this->dataPtr->inspectionResult = -1;
      if (submit_srv.response.success)
      {
        this->dataPtr->inspectionResult = submit_srv.response.inspection_result;
      }

      // Trigger the delivery animation
      this->dataPtr->collectAnimation->SetTime(0);
      this->dataPtr->model->SetAnimation(this->dataPtr->collectAnimation);
      ROS_INFO_STREAM("drone successfully triggered.");
      this->dataPtr->currentState = "collecting";
    }
    this->dataPtr->deliveryTriggered = false;
  }
  if (this->dataPtr->currentState == "collecting")
  {
    bool collectAnimationDone = this->dataPtr->collectAnimation->GetTime() >= \
      this->dataPtr->collectAnimation->GetLength();
    if (collectAnimationDone)
    {
      // Dispose of the real box in favour of the dummy collected box visual.
      gazebo::msgs::GzString activateMsg;
      activateMsg.set_data("activate_once");
      this->dataPtr->clearBoxesPub->Publish(activateMsg);

      // Enable the collected box visual.
      gazebo::msgs::GzString toggleMsg;
      toggleMsg.set_data("on");
      this->dataPtr->toggleBoxVisualPub->Publish(toggleMsg);

      gzdbg << "Collect animation finished." << std::endl;
      this->dataPtr->currentState = "collected";
    }
  }
  if (this->dataPtr->currentState == "collected")
  {
    // Report the result of the previously-performed shipment inspection.
    if (this->dataPtr->inspectionResult < 0)
    {
      ROS_ERROR_STREAM("Failed to submit shipment for inspection.");
    }
    else
    {
      ROS_INFO_STREAM("Result of inspection: " << this->dataPtr->inspectionResult);
    }

    // Trigger the return animation.
    this->dataPtr->returnAnimation->SetTime(0);
    this->dataPtr->model->SetAnimation(this->dataPtr->returnAnimation);
    this->dataPtr->currentState = "returning";
  }
  if (this->dataPtr->currentState == "returning")
  {
    bool returnAnimationDone = this->dataPtr->returnAnimation->GetTime() >= \
      this->dataPtr->returnAnimation->GetLength();
    if (returnAnimationDone)
    {
      gzdbg << "Return animation finished." << std::endl;

      // Disable the visual of the dummy collected box.
      gazebo::msgs::GzString toggleMsg;
      toggleMsg.set_data("off");
      this->dataPtr->toggleBoxVisualPub->Publish(toggleMsg);

      this->dataPtr->currentState = "ready_to_collect";
    }
  }
  std_msgs::String stateMsg;
  stateMsg.data = this->dataPtr->currentState;
  this->dataPtr->statePub.publish(stateMsg);

}

/////////////////////////////////////////////////
bool ROSDronePlugin::OnCommand(
  osrf_gear::DroneControl::Request &_req,
  osrf_gear::DroneControl::Response &_res)
{
  if (this->dataPtr->currentState != "ready_to_collect")
  {
    ROS_ERROR_STREAM("Drone not successfully triggered as it was not ready to collect shipping boxes.");
    _res.success = false;
    return true;
  }
  if (this->dataPtr->waitingBoxName == "")
  {
    ROS_ERROR_STREAM("Drone not successfully triggered as there is no shipping box to collect.");
    _res.success = false;
    return true;
  }
  ROS_INFO_STREAM("Drone collection triggered for shipment: " << _req.shipment_type);
  this->dataPtr->shipmentType = _req.shipment_type;
  this->dataPtr->deliveryTriggered = true;
  _res.success = true;
  return true;
}

/////////////////////////////////////////////////
void ROSDronePlugin::OnWaitingBox(ConstGzStringPtr &_msg)
{
  this->dataPtr->waitingBoxName = _msg->data();
}
