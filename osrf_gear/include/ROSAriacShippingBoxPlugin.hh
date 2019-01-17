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
/*
 * Desc: shipping box plugin
 * Author: Deanna Hood
 */
#ifndef _GAZEBO_SHIPPING_BOX_PLUGIN_HH_
#define _GAZEBO_SHIPPING_BOX_PLUGIN_HH_

#include <string>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <osrf_gear/ARIAC.hh>
#include "SideContactPlugin.hh"
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  /// \brief A plugin for a contact sensor on a shipping box.
  class GAZEBO_VISIBLE ShippingBoxPlugin : public SideContactPlugin
  {
    /// \brief Constructor.
    public: ShippingBoxPlugin();

    /// \brief Destructor.
    public: virtual ~ShippingBoxPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the world update event
    protected: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Update the shipment based on which models are in contact
    protected: void ProcessContactingModels();

    /// \brief Create a fixed joint to all contacting models
    protected: virtual void LockContactingModels();

    /// \brief Remove any fixed joints to contacting models
    protected: virtual void UnlockContactingModels();

    /// \brief Update the shipment based on which models are in contact
    public: std::string DetermineModelType(const std::string &modelName);

    /// \brief Callback for when a new subscriber connects to the Shipment ROS publisher
    /// This will check that only the /gazebo node is subscribed during the competition
    protected: void OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub);

    /// \brief Publish the Shipment ROS message
    protected: void PublishShipmentMsg();

    /// \brief Service for locking the models to the shipping box and disabling updates
    protected: void HandleLockModelsRequest(ConstGzStringPtr &_msg);

    /// \brief Service for clearing the shipping box
    protected: bool HandleClearService(
      ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response>& event);

    /// \brief Shipment which is currently on the shipping box
    protected: ariac::Shipment currentShipment;

    /// \brief ID of shipping box
    protected: std::string shippingBoxID;

    /// \brief Fixed joints to lock contacting models
    protected: std::vector<physics::JointPtr> fixedJoints;

    /// \brief ROS node handle
    protected: ros::NodeHandle *rosNode;

    /// \brief Gazebo node for communication
    protected: transport::NodePtr gzNode;

    /// \brief Publisher for the shipment state
    protected: ros::Publisher currentShipmentPub;

    /// \brief Whether or not the Shipment ROS topic is enabled
    /// If unpermitted subscribers connect during the competition, publishing is disabled
    protected: bool publishingEnabled;

    /// \brief Whether or not the Shipment will be used in a nested animation (requires workarounds).
    protected: bool nestedAnimation = false;

    /// \brief Whether or not the Shipment will lock toggle its visuals at a particular pose.
    protected: bool toggleVisualsAtPose = false;

    /// \brief The pose at which visuals should be toggled.
    protected: ignition::math::Vector3d toggleVisualsAt;

    /// \brief Publisher for toggling the visuals.
    protected: transport::PublisherPtr toggleVisualsPub;

    /// \brief Whether or not the Shipment will clear the models at a particular pose.
    protected: bool clearModelsAtPose = false;

    /// \brief The pose at which models should be cleared.
    protected: ignition::math::Vector3d clearModelsAt;

    /// \brief Service that locks models to the shipping box
    public: ros::ServiceServer lockModelsServer;

    /// \brief ROS service that clears the shipping box
    public: ros::ServiceServer clearShippingBoxServer;

    /// \brief Products to ignore (will be published as faulty in shipping box msgs)
    /// The namespace of the product (e.g. bin7) is ignored.
    /// e.g. if model_name1 is faulty, either bin7|model_name1 or bin6|model_name1 will be considered faulty
    protected: std::vector<std::string> faultyProductNames;

    /// \brief Gazebo subscriber to the lock models topic
    protected: transport::SubscriberPtr lockModelsSub;

    /// \brief Whether or not the animation will be triggered at a particular pose.
    protected: bool triggerAnimationAtPose = false;

    /// \brief The pose at which the animation should be triggered.
    protected: ignition::math::Vector3d triggerAnimationAt;

    /// \brief The pose at which the animation should end.
    protected: ignition::math::Vector3d endAnimationAt;

    /// \brief Animation for going down the ramp
    public: gazebo::common::PoseAnimationPtr rampAnimation;
  };
}
#endif
