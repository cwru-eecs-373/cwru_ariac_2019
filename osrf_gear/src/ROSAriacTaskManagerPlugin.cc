/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <ostream>
#include <string>
#include <thread>
#include <vector>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/LogRecord.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>


#include "osrf_gear/ARIAC.hh"
#include "osrf_gear/ROSAriacTaskManagerPlugin.hh"
#include "osrf_gear/AriacScorer.h"
#include "osrf_gear/DetectShipment.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/VacuumGripperState.h"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSAriacTaskManagerPlugin class.
  struct ROSAriacTaskManagerPluginPrivate
  {
    /// \brief World pointer.
    public: physics::WorldPtr world;

    /// \brief SDF pointer.
    public: sdf::ElementPtr sdf;

    /// \brief Collection of orders to announce.
    public: std::vector<ariac::Order> ordersToAnnounce;

    /// \brief Collection of orders which have been announced but are not yet complete.
    /// The order at the top of the stack is the active order.
    public: std::stack<ariac::Order> ordersInProgress;

    /// \brief Mapping between material types and their locations.
    public: std::map<std::string, std::vector<std::string> > materialLocations;

    /// \brief A scorer to mange the game score.
    public: AriacScorer ariacScorer;

    /// \brief The current game score.
    public: ariac::GameScore currentGameScore;

    /// \brief ROS node handle.
    public: std::unique_ptr<ros::NodeHandle> rosnode;

    /// \brief Publishes an order.
    public: ros::Publisher orderPub;

    /// \brief ROS subscriber for the shipping box states.
    public: ros::Subscriber shippingBoxInfoSub;

    /// \brief ROS subscribers for the gripper state.
    public: ros::Subscriber gripper1StateSub;
    public: ros::Subscriber gripper2StateSub;

    /// \brief Publishes the Gazebo task state.
    public: ros::Publisher taskStatePub;

    /// \brief Publishes the game score total.
    public: ros::Publisher taskScorePub;

    /// \brief Name of service that allows the user to start the competition.
    public: std::string compStartServiceName;

    /// \brief Service that allows the user to start the competition.
    public: ros::ServiceServer compStartServiceServer;

    /// \brief Service that allows the user to end the competition.
    public: ros::ServiceServer compEndServiceServer;

    /// \brief Service that allows users to query the location of materials.
    public: ros::ServiceServer getMaterialLocationsServiceServer;

    /// \brief Service that allows a tray to be submitted for inspection.
    public: ros::ServiceServer submitTrayServiceServer;

    /// \brief Map of agv id to server that handles requests to deliver shipment
    public: std::map<int, ros::ServiceServer> agvDeliverService;

    /// \brief Map of agv id to client that can get its content
    public: std::map<int, ros::ServiceClient> agvGetContentClient;

    /// \brief Map of agv id to client that can ask AGV to animate
    public: std::map<int, ros::ServiceClient> agvAnimateClient;

    /// \brief Transportation node.
    public: transport::NodePtr node;

    /// \brief Publisher for enabling the product population on the conveyor.
    public: transport::PublisherPtr populatePub;

    /// \brief Publisher for enabling the conveyor.
    public: transport::PublisherPtr conveyorEnablePub;

    /// \brief Publisher for controlling the blackout of sensors.
    public: transport::PublisherPtr sensorBlackoutControlPub;

    /// \brief Duration at which to blackout sensors.
    public: double sensorBlackoutDuration;

    /// \brief Product count at which to blackout sensors.
    public: int sensorBlackoutProductCount = 0;

    /// \brief If sensor blackout is currently in progress.
    public: bool sensorBlackoutInProgress = false;

    /// \brief The start time of the sensor blackout.
    public: common::Time sensorBlackoutStartTime;

    /// \brief Timer for regularly publishing state/score.
    public: ros::Timer statusPubTimer;

    /// \brief Connection event.
    public: event::ConnectionPtr connection;

    /// \brief Publish Gazebo server control messages.
    public: transport::PublisherPtr serverControlPub;

    /// \brief The time the last update was called.
    public: common::Time lastUpdateTime;

    /// \brief The time the sim time was last published.
    public: common::Time lastSimTimePublish;

    /// \brief The time specified in the product is relative to this time.
    public: common::Time gameStartTime;

    /// \brief The time in seconds permitted to complete the trial.
    public: double timeLimit;

    /// \brief The time in seconds that has been spent on the current order.
    public: double timeSpentOnCurrentOrder;

    /// \brief Pointer to the current state.
    public: std::string currentState = "init";

    /// \brief A mutex to protect currentState.
    public: std::mutex mutex;

    // During the competition, this environment variable will be set.
    bool competitionMode = false;
  };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ROSAriacTaskManagerPlugin)

/////////////////////////////////////////////////
static void fillOrderMsg(const ariac::Order &_order,
                        osrf_gear::Order &_msgOrder)
{
  _msgOrder.order_id = _order.orderID;
  for (const auto &shipment : _order.shipments)
  {
    osrf_gear::Shipment msgShipment;
    msgShipment.shipment_type = shipment.shipmentType;
    for (const auto &obj : shipment.products)
    {
      osrf_gear::Product msgObj;
      msgObj.type = obj.type;
      msgObj.pose.position.x = obj.pose.Pos().X();
      msgObj.pose.position.y = obj.pose.Pos().Y();
      msgObj.pose.position.z = obj.pose.Pos().Z();
      msgObj.pose.orientation.x = obj.pose.Rot().X();
      msgObj.pose.orientation.y = obj.pose.Rot().Y();
      msgObj.pose.orientation.z = obj.pose.Rot().Z();
      msgObj.pose.orientation.w = obj.pose.Rot().W();

      // Add the product to the shipment.
      msgShipment.products.push_back(msgObj);
    }
    _msgOrder.shipments.push_back(msgShipment);
  }
}

/////////////////////////////////////////////////
ROSAriacTaskManagerPlugin::ROSAriacTaskManagerPlugin()
  : dataPtr(new ROSAriacTaskManagerPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSAriacTaskManagerPlugin::~ROSAriacTaskManagerPlugin()
{
  this->dataPtr->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::Load(physics::WorldPtr _world,
  sdf::ElementPtr _sdf)
{
  gzdbg << "ARIAC VERSION: 2.1.8\n";
  auto competitionEnv = std::getenv("ARIAC_COMPETITION");
  this->dataPtr->competitionMode = competitionEnv != NULL;
  gzdbg << "ARIAC COMPETITION MODE: " << (this->dataPtr->competitionMode ? competitionEnv : "false") << std::endl;

  GZ_ASSERT(_world, "ROSAriacTaskManagerPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "ROSAriacTaskManagerPlugin sdf pointer is NULL");
  this->dataPtr->world = _world;
  this->dataPtr->sdf = _sdf;

  // Initialize Gazebo transport.
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  std::string robotNamespace = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    robotNamespace = _sdf->GetElement(
      "robot_namespace")->Get<std::string>() + "/";
  }

  // Initialize ROS
  this->dataPtr->rosnode.reset(new ros::NodeHandle(robotNamespace));

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  this->dataPtr->timeLimit = -1.0;
  if (_sdf->HasElement("competition_time_limit"))
    this->dataPtr->timeLimit = _sdf->Get<double>("competition_time_limit");

  std::string compEndServiceName = "end_competition";
  if (_sdf->HasElement("end_competition_service_name"))
    compEndServiceName = _sdf->Get<std::string>("end_competition_service_name");

  this->dataPtr->compStartServiceName = "start_competition";
  if (_sdf->HasElement("start_competition_service_name"))
    this->dataPtr->compStartServiceName = _sdf->Get<std::string>("start_competition_service_name");

  std::string taskStateTopic = "competition_state";
  if (_sdf->HasElement("task_state_topic"))
    taskStateTopic = _sdf->Get<std::string>("task_state_topic");

  std::string taskScoreTopic = "current_score";
  if (_sdf->HasElement("task_score_topic"))
    taskScoreTopic = _sdf->Get<std::string>("task_score_topic");

  std::string conveyorEnableTopic = "conveyor/enable";
  if (_sdf->HasElement("conveyor_enable_topic"))
    conveyorEnableTopic = _sdf->Get<std::string>("conveyor_enable_topic");

  std::string populationActivateTopic = "populate_belt";
  if (_sdf->HasElement("population_activate_topic"))
    populationActivateTopic = _sdf->Get<std::string>("population_activate_topic");

  std::string ordersTopic = "orders";
  if (_sdf->HasElement("orders_topic"))
    ordersTopic = _sdf->Get<std::string>("orders_topic");

  std::string submitTrayServiceName = "submit_tray";
  if (_sdf->HasElement("submit_tray_service_name"))
    submitTrayServiceName = _sdf->Get<std::string>("submit_tray_service_name");

  std::string getMaterialLocationsServiceName = "material_locations";
  if (_sdf->HasElement("material_locations_service_name"))
    getMaterialLocationsServiceName = _sdf->Get<std::string>("material_locations_service_name");

  std::map<int, std::string> agvDeliverServiceName;
  std::map<int, std::string> agvAnimateServiceName;
  std::map<int, std::string> agvGetContentServiceName;
  if (_sdf->HasElement("agv"))
  {
    sdf::ElementPtr agvElem = _sdf->GetElement("agv");
    while (agvElem)
    {
      int index = agvElem->Get<int>("index");
      agvDeliverServiceName[index] = "deliver";
      agvAnimateServiceName[index] = "animate";
      agvGetContentServiceName[index] = "get_content";
      if (agvElem->HasElement("agv_control_service_name"))
      {
        agvDeliverServiceName[index] = agvElem->Get<std::string>("agv_control_service_name");
      }
      if (agvElem->HasElement("agv_animate_service_name"))
      {
        agvAnimateServiceName[index] = agvElem->Get<std::string>("agv_animate_service_name");
      }
      if (agvElem->HasElement("get_content_service_name"))
      {
        agvGetContentServiceName[index] = agvElem->Get<std::string>("get_content_service_name");
      }

      agvElem = agvElem->GetNextElement("agv");
    }
  }


  // Parse the orders.
  sdf::ElementPtr orderElem = NULL;
  if (_sdf->HasElement("order"))
  {
    orderElem = _sdf->GetElement("order");
  }

  while (orderElem)
  {
    // Parse the order name.
    ariac::OrderID_t orderID = orderElem->Get<std::string>("name");

    // Parse the start time.
    double startTime = std::numeric_limits<double>::infinity();
    if (orderElem->HasElement("start_time"))
    {
      sdf::ElementPtr startTimeElement = orderElem->GetElement("start_time");
      startTime = startTimeElement->Get<double>();
    }

    // Parse the interruption criteria.
    int interruptOnUnwantedProducts = -1;
    if (orderElem->HasElement("interrupt_on_unwanted_products"))
    {
      sdf::ElementPtr interruptOnUnwantedProductsElem = orderElem->GetElement("interrupt_on_unwanted_products");
      interruptOnUnwantedProducts = interruptOnUnwantedProductsElem->Get<int>();
    }
    int interruptOnWantedProducts = -1;
    if (orderElem->HasElement("interrupt_on_wanted_products"))
    {
      sdf::ElementPtr interruptOnWantedProductsElem = orderElem->GetElement("interrupt_on_wanted_products");
      interruptOnWantedProducts = interruptOnWantedProductsElem->Get<int>();
    }

    // Parse the allowed completion time.
    double allowedTime = std::numeric_limits<double>::infinity();
    if (orderElem->HasElement("allowed_time"))
    {
      sdf::ElementPtr allowedTimeElement = orderElem->GetElement("allowed_time");
      allowedTime = allowedTimeElement->Get<double>();
    }

    // Parse the shipments.
    if (!orderElem->HasElement("shipment"))
    {
      gzerr << "Unable to find <shipment> element in <order>. Ignoring" << std::endl;
      orderElem = orderElem->GetNextElement("order");
      continue;
    }

    // Store all shipments for an order.
    std::vector<ariac::Shipment> shipments;

    sdf::ElementPtr shipmentElem = orderElem->GetElement("shipment");
    while (shipmentElem)
    {
      // Check the validity of the shipment.
      if (!shipmentElem->HasElement("product"))
      {
        gzerr << "Unable to find <product> element in <shipment>. Ignoring"
              << std::endl;
        shipmentElem = shipmentElem->GetNextElement("shipment");
        continue;
      }

      ariac::Shipment shipment;

      // Parse the shipment type.
      ariac::ShipmentType_t shipmentType;
      if (shipmentElem->HasElement("shipment_type"))
      {
        shipmentType = shipmentElem->Get<std::string>("shipment_type");
      }
      shipment.shipmentType = shipmentType;

      // Parse the products inside the shipment.
      sdf::ElementPtr productElem = shipmentElem->GetElement("product");
      while (productElem)
      {
        // Parse the product type.
        if (!productElem->HasElement("type"))
        {
          gzerr << "Unable to find <type> in product.\n";
          productElem = productElem->GetNextElement("product");
          continue;
        }
        sdf::ElementPtr typeElement = productElem->GetElement("type");
        std::string type = typeElement->Get<std::string>();

        // Parse the product pose (optional).
        if (!productElem->HasElement("pose"))
        {
          gzerr << "Unable to find <pose> in product.\n";
          productElem = productElem->GetNextElement("product");
          continue;
        }
        sdf::ElementPtr poseElement = productElem->GetElement("pose");
        ignition::math::Pose3d pose = poseElement->Get<ignition::math::Pose3d>();

        // Add the product to the shipment.
        bool isFaulty = false;  // We never want to request faulty products.
        ariac::Product obj = {type, isFaulty, pose};
        shipment.products.push_back(obj);

        productElem = productElem->GetNextElement("product");
      }

      // Add a new shipment to the collection of shipments.
      shipments.push_back(shipment);

      shipmentElem = shipmentElem->GetNextElement("shipment");
    }

    // Add a new order.
    ariac::Order order = {orderID, startTime, interruptOnUnwantedProducts, interruptOnWantedProducts, allowedTime, shipments, 0.0};
    this->dataPtr->ordersToAnnounce.push_back(order);

    orderElem = orderElem->GetNextElement("order");
  }

  // Sort the orders by their start times.
  std::sort(this->dataPtr->ordersToAnnounce.begin(), this->dataPtr->ordersToAnnounce.end());

  // Debug output.
  // gzdbg << "Orders:" << std::endl;
  // for (auto order : this->dataPtr->ordersToAnnounce)
  //   gzdbg << order << std::endl;

  // Parse the material storage locations.
  if (_sdf->HasElement("material_locations"))
  {
    sdf::ElementPtr materialLocationsElem = _sdf->GetElement("material_locations");
    sdf::ElementPtr materialElem = NULL;
    if (materialLocationsElem->HasElement("material"))
    {
      materialElem = materialLocationsElem->GetElement("material");
    }
    while (materialElem)
    {
      std::string materialType = materialElem->Get<std::string>("type");
      std::vector<std::string> locations;

      // Parse locations of this material.
      sdf::ElementPtr locationElem = NULL;
      if (materialElem->HasElement("location"))
      {
        locationElem = materialElem->GetElement("location");
      }
      while (locationElem)
      {
        std::string location = locationElem->Get<std::string>("storage_unit");
        locations.push_back(location);
        locationElem = locationElem->GetNextElement("location");
      }
      this->dataPtr->materialLocations[materialType] = locations;
      materialElem = materialElem->GetNextElement("material");
    }
  }

  if (_sdf->HasElement("sensor_blackout"))
  {
    auto sensorBlackoutElem = _sdf->GetElement("sensor_blackout");
    std::string sensorEnableTopic = sensorBlackoutElem->Get<std::string>("topic");
    this->dataPtr->sensorBlackoutProductCount = sensorBlackoutElem->Get<int>("product_count");
    this->dataPtr->sensorBlackoutDuration = sensorBlackoutElem->Get<double>("duration");
    this->dataPtr->sensorBlackoutControlPub =
      this->dataPtr->node->Advertise<msgs::GzString>(sensorEnableTopic);
  }

  // Publisher for announcing new orders.
  this->dataPtr->orderPub = this->dataPtr->rosnode->advertise<
    osrf_gear::Order>(ordersTopic, 1000, true);  // latched=true

  // Publisher for announcing new state of the competition.
  this->dataPtr->taskStatePub = this->dataPtr->rosnode->advertise<
    std_msgs::String>(taskStateTopic, 1000);

  // Publisher for announcing the score of the game.
  if (!this->dataPtr->competitionMode)
  {
    this->dataPtr->taskScorePub = this->dataPtr->rosnode->advertise<
      std_msgs::Float32>(taskScoreTopic, 1000);
  }

  // Service for ending the competition.
  this->dataPtr->compEndServiceServer =
    this->dataPtr->rosnode->advertiseService(compEndServiceName,
      &ROSAriacTaskManagerPlugin::HandleEndService, this);

  // Service for submitting AGV trays for inspection.
  this->dataPtr->submitTrayServiceServer =
    this->dataPtr->rosnode->advertiseService(submitTrayServiceName,
      &ROSAriacTaskManagerPlugin::HandleSubmitTrayService, this);

  // Service for querying material storage locations.
  if (!this->dataPtr->competitionMode)
  {
    this->dataPtr->getMaterialLocationsServiceServer =
      this->dataPtr->rosnode->advertiseService(getMaterialLocationsServiceName,
        &ROSAriacTaskManagerPlugin::HandleGetMaterialLocationsService, this);
  }

  // Publisher for the conveyor enable topic
  this->dataPtr->conveyorEnablePub =
    this->dataPtr->node->Advertise<msgs::GzString>(conveyorEnableTopic);

  // Timer for regularly publishing state/score.
  this->dataPtr->statusPubTimer =
    this->dataPtr->rosnode->createTimer(ros::Duration(0.1),
      &ROSAriacTaskManagerPlugin::PublishStatus, this);

  this->dataPtr->populatePub =
    this->dataPtr->node->Advertise<msgs::GzString>(populationActivateTopic);

  for (auto & pair : agvDeliverServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->dataPtr->agvDeliverService[index] =
      this->dataPtr->rosnode->advertiseService<osrf_gear::AGVControl::Request, osrf_gear::AGVControl::Response>(
        serviceName,
        boost::bind(&ROSAriacTaskManagerPlugin::HandleAGVDeliverService, this,
        _1, _2, index));
  }

  for (auto & pair : agvGetContentServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->dataPtr->agvGetContentClient[index] =
      this->dataPtr->rosnode->serviceClient<osrf_gear::DetectShipment>(serviceName);
  }

  for (auto & pair : agvAnimateServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->dataPtr->agvAnimateClient[index] =
      this->dataPtr->rosnode->serviceClient<std_srvs::Trigger>(serviceName);
  }

  this->dataPtr->serverControlPub =
    this->dataPtr->node->Advertise<msgs::ServerControl>("/gazebo/server/control");

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
    boost::bind(&ROSAriacTaskManagerPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto currentSimTime = this->dataPtr->world->SimTime();

  // Delay advertising the competition start service to avoid a crash.
  // Sometimes if the competition is started before the world is fully loaded, it causes a crash.
  // See https://bitbucket.org/osrf/ariac/issues/91
  if (!this->dataPtr->compStartServiceServer && currentSimTime.Double() >= 5.0)
  {
    // Service for starting the competition.
    this->dataPtr->compStartServiceServer =
      this->dataPtr->rosnode->advertiseService(this->dataPtr->compStartServiceName,
        &ROSAriacTaskManagerPlugin::HandleStartService, this);
  }

  if ((currentSimTime - this->dataPtr->lastSimTimePublish).Double() >= 1.0)
  {
    gzdbg << "Sim time: " << currentSimTime.Double() << std::endl;
    this->dataPtr->lastSimTimePublish = currentSimTime;
  }

  double elapsedTime = (currentSimTime - this->dataPtr->lastUpdateTime).Double();
  if (this->dataPtr->timeLimit >= 0 && this->dataPtr->currentState == "go" &&
    (currentSimTime - this->dataPtr->gameStartTime) > this->dataPtr->timeLimit)
  {
    this->dataPtr->currentState = "end_game";
  }

  if (this->dataPtr->currentState == "ready")
  {
    this->dataPtr->gameStartTime = currentSimTime;
    this->dataPtr->currentState = "go";

    this->EnableConveyorBeltControl();
    this->PopulateConveyorBelt();
  }
  else if (this->dataPtr->currentState == "go")
  {

    // Update the order manager.
    this->ProcessOrdersToAnnounce(currentSimTime);

    // Update the sensors if appropriate.
    this->ProcessSensorBlackout();

    // Update the score.
    // TODO(sloretz) only publish this when an event that could change the score happens
    auto gameScore = this->dataPtr->ariacScorer.GetGameScore();
    if (gameScore.total() != this->dataPtr->currentGameScore.total())
    {
      std::ostringstream logMessage;
      logMessage << "Current game score: " << gameScore.total();
      ROS_DEBUG_STREAM(logMessage.str().c_str());
      gzdbg << logMessage.str() << std::endl;
      this->dataPtr->currentGameScore = gameScore;
    }

    if (!this->dataPtr->ordersInProgress.empty())
    {
      this->dataPtr->ordersInProgress.top().timeTaken += elapsedTime;
      auto orderID = this->dataPtr->ordersInProgress.top().orderID;
      // TODO: timing should probably be managed by the scorer but we want to use sim time
      this->dataPtr->timeSpentOnCurrentOrder = this->dataPtr->ordersInProgress.top().timeTaken;

      // Check for completed orders.
      bool orderCompleted = gameScore.orderScores.at(orderID).isComplete();
      if (orderCompleted)
      {
        std::ostringstream logMessage;
        logMessage << "Order complete: " << orderID;
        ROS_INFO_STREAM(logMessage.str().c_str());
        gzdbg << logMessage.str() << std::endl;
        this->StopCurrentOrder();
      }
      else
      {
        // Check if the time limit for the current order has been exceeded.
        if (this->dataPtr->timeSpentOnCurrentOrder > this->dataPtr->ordersInProgress.top().allowedTime)
        {
          std::ostringstream logMessage;
          logMessage << "Order timed out: " << orderID;
          ROS_INFO_STREAM(logMessage.str().c_str());
          gzdbg << logMessage.str() << std::endl;
          this->StopCurrentOrder();
        }
      }
    }

    if (this->dataPtr->ordersInProgress.empty() && this->dataPtr->ordersToAnnounce.empty())
    {
      gzdbg << "No more orders to process." << std::endl;
      this->dataPtr->currentState = "end_game";
    }
  }
  else if (this->dataPtr->currentState == "end_game")
  {
    this->dataPtr->currentGameScore = this->dataPtr->ariacScorer.GetGameScore();
    if (this->dataPtr->gameStartTime != common::Time())
    {
      this->dataPtr->currentGameScore.totalProcessTime =
        (currentSimTime - this->dataPtr->gameStartTime).Double();
    }
    std::ostringstream logMessage;
    logMessage << "End of trial. Final score: " << \
      this->dataPtr->currentGameScore.total() << "\nScore breakdown:\n" << \
      this->dataPtr->currentGameScore;
    ROS_INFO_STREAM(logMessage.str().c_str());
    gzdbg << logMessage.str() << std::endl;
    this->dataPtr->currentState = "done";

    auto v = std::getenv("ARIAC_EXIT_ON_COMPLETION");
    if (v)
    {
      // Gazebo will accumulate a number of state loggings before writing them to file.
      // Request that the accumulated states are written now before we shutdown,
      // otherwise the end of the state log may be cut off.
      util::LogRecord::Instance()->Notify();
      std::this_thread::sleep_for(std::chrono::seconds(2));

      std::string logMessage = "Requesting that Gazebo shut down";
      gzmsg << logMessage << std::endl;
      ROS_INFO("%s", logMessage.c_str());
      msgs::ServerControl msg;
      msg.set_stop(true);
      this->dataPtr->serverControlPub->Publish(msg);
      gazebo::shutdown();
    }
  }

  this->dataPtr->lastUpdateTime = currentSimTime;
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::PublishStatus(const ros::TimerEvent&)
{
  std_msgs::Float32 scoreMsg;
  scoreMsg.data = this->dataPtr->currentGameScore.total();
  if (!this->dataPtr->competitionMode)
  {
    this->dataPtr->taskScorePub.publish(scoreMsg);
  }

  std_msgs::String stateMsg;
  stateMsg.data = this->dataPtr->currentState;
  this->dataPtr->taskStatePub.publish(stateMsg);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::ProcessOrdersToAnnounce(gazebo::common::Time simTime)
{
  if (this->dataPtr->ordersToAnnounce.empty())
    return;

  auto nextOrder = this->dataPtr->ordersToAnnounce.front();
  bool interruptOnUnwantedProducts = nextOrder.interruptOnUnwantedProducts > 0;
  bool interruptOnWantedProducts = nextOrder.interruptOnWantedProducts > 0;
  bool noActiveOrder = this->dataPtr->ordersInProgress.empty();
  auto elapsed = this->dataPtr->world->SimTime() - this->dataPtr->gameStartTime;
  bool announceNextOrder = false;

  // Check whether announce a new order from the list.
  // Announce next order if the appropriate amount of time has elapsed
  announceNextOrder |= elapsed.Double() >= nextOrder.startTime;
  // Announce next order if there is no active order and we are waiting to interrupt
  announceNextOrder |= noActiveOrder && (interruptOnWantedProducts || interruptOnUnwantedProducts);

  int maxNumUnwantedProducts = 0;
  int maxNumWantedProducts = 0;
  // Check if it's time to interrupt (skip if we're already interrupting anyway)
  if (!announceNextOrder && (interruptOnWantedProducts || interruptOnUnwantedProducts))
  {
    // Check if the products in the shipping boxes are enough to interrupt the current order

    // Determine what products are in the next order
    std::vector<std::string> productsInNextOrder;
    for (const auto & shipment : nextOrder.shipments)
    {
      for (const auto & product : shipment.products)
      {
        productsInNextOrder.push_back(product.type);
      }
    }

    // Check the shipping boxes for products from the pending order
    /*
    std::vector<int> numUnwantedProductsInShippingBoxes;
    std::vector<int> numWantedProductsInShippingBoxes;
    for (const auto & shippingBox : this->dataPtr->ariacScorer.GetShippingBoxes())
    {
      int numUnwantedProductsInShippingBox = 0;
      int numWantedProductsInShippingBox = 0;
      std::vector<std::string> productsInNextOrder_copy(productsInNextOrder);
      for (const auto product : shippingBox.currentShipment.products)
      {
        // Don't count faulty products, because they have to be removed anyway.
        if (product.isFaulty)
        {
          continue;
        }
        auto it = std::find(productsInNextOrder_copy.begin(), productsInNextOrder_copy.end(), product.type);
        if (it == productsInNextOrder_copy.end())
        {
          numUnwantedProductsInShippingBox += 1;
        }
        else
        {
          numWantedProductsInShippingBox += 1;
          productsInNextOrder_copy.erase(it);
        }
      }
      numUnwantedProductsInShippingBoxes.push_back(numUnwantedProductsInShippingBox);
      numWantedProductsInShippingBoxes.push_back(numWantedProductsInShippingBox);
    }
    maxNumUnwantedProducts = *std::max_element(numUnwantedProductsInShippingBoxes.begin(), numUnwantedProductsInShippingBoxes.end());
    maxNumWantedProducts = *std::max_element(numWantedProductsInShippingBoxes.begin(), numWantedProductsInShippingBoxes.end());

    // Announce next order if the appropriate number of wanted/unwanted products are detected
    announceNextOrder |= interruptOnWantedProducts && (maxNumWantedProducts >= nextOrder.interruptOnWantedProducts);
    announceNextOrder |= interruptOnUnwantedProducts && (maxNumUnwantedProducts >= nextOrder.interruptOnUnwantedProducts);
    */
  }

  if (announceNextOrder)
  {
    auto updateLocn = nextOrder.orderID.find("_update");
    if (updateLocn != std::string::npos)
    {
      gzdbg << "Order to update: " << nextOrder.orderID << std::endl;
      this->AnnounceOrder(nextOrder);

      // Update the order the scorer's monitoring
      gzdbg << "Updating order: " << nextOrder << std::endl;
      auto nextOrderID = nextOrder.orderID.substr(0, updateLocn);
      osrf_gear::Order orderMsg;
      fillOrderMsg(nextOrder, orderMsg);
      this->dataPtr->ariacScorer.NotifyOrderUpdated(simTime, nextOrderID, orderMsg);
      this->dataPtr->ordersToAnnounce.erase(this->dataPtr->ordersToAnnounce.begin());
      return;
    }

    gzdbg << "New order to announce: " << nextOrder.orderID << std::endl;

    // Move order to the 'in process' stack
    this->dataPtr->ordersInProgress.push(ariac::Order(nextOrder));
    this->dataPtr->ordersToAnnounce.erase(this->dataPtr->ordersToAnnounce.begin());

    this->AnnounceOrder(nextOrder);
    // Assign the scorer the order to monitor
    gzdbg << "Assigning order: " << nextOrder << std::endl;
    osrf_gear::Order orderMsg;
    fillOrderMsg(nextOrder, orderMsg);
    this->dataPtr->ariacScorer.NotifyOrderStarted(simTime, orderMsg);
  }
}


/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::ProcessSensorBlackout()
{
  auto currentSimTime = this->dataPtr->world->SimTime();
  if (this->dataPtr->sensorBlackoutProductCount > 0)
  {
    // Count total products in all boxes.
    int totalProducts = 0;
    /*
    for (const auto & shippingBox : this->dataPtr->ariacScorer.GetShippingBoxes())
    {
      totalProducts += shippingBox.currentShipment.products.size();
    }
    if (totalProducts >= this->dataPtr->sensorBlackoutProductCount)
    {
      gzdbg << "Triggering sensor blackout because " << totalProducts << " products detected." << std::endl;
      gazebo::msgs::GzString activateMsg;
      activateMsg.set_data("deactivate");
      this->dataPtr->sensorBlackoutControlPub->Publish(activateMsg);
      this->dataPtr->sensorBlackoutProductCount = -1;
      this->dataPtr->sensorBlackoutStartTime = currentSimTime;
      this->dataPtr->sensorBlackoutInProgress = true;
    }
    */
  }
  if (this->dataPtr->sensorBlackoutInProgress)
  {
    auto elapsedTime = (currentSimTime - this->dataPtr->sensorBlackoutStartTime).Double();
    if (elapsedTime > this->dataPtr->sensorBlackoutDuration)
    {
      gzdbg << "Ending sensor blackout." << std::endl;
      gazebo::msgs::GzString activateMsg;
      activateMsg.set_data("activate");
      this->dataPtr->sensorBlackoutControlPub->Publish(activateMsg);
      this->dataPtr->sensorBlackoutInProgress = false;
    }
  }
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleStartService(
  std_srvs::Trigger::Request & req,
  std_srvs::Trigger::Response & res)
{
  gzdbg << "Handle start service called\n";
  (void)req;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->currentState == "init") {
    this->dataPtr->currentState = "ready";
    res.success = true;
    res.message = "competition started successfully!";
    return true;
  }
  res.success = false;
  res.message = "cannot start if not in 'init' state";
  return true;
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleEndService(
  std_srvs::Trigger::Request & req,
  std_srvs::Trigger::Response & res)
{
  gzdbg << "Handle end service called\n";
  (void)req;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->currentState = "end_game";
  res.success = true;
  res.message = "competition ended successfully!";
  return true;
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleSubmitTrayService(
  ros::ServiceEvent<osrf_gear::SubmitTray::Request, osrf_gear::SubmitTray::Response> & event)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const osrf_gear::SubmitTray::Request& req = event.getRequest();
  osrf_gear::SubmitTray::Response& res = event.getResponse();

  const std::string& callerName = event.getCallerName();
  gzdbg << "Submit tray service called by: " << callerName << std::endl;

  if (this->dataPtr->competitionMode && callerName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition mode is enabled so this service is not enabled.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    res.success = false;
    return true;
  }

  if (this->dataPtr->currentState != "go") {
    std::string errStr = "Competition is not running so shipping boxes cannot be submitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return false;
  }

  // ariac::ShippingBox shippingBox;
  // gzdbg << "SubmitShipment request received for shipping box: " << req.shipping_box_id << std::endl;
  // if (!this->dataPtr->ariacScorer.GetShippingBoxById(req.shipping_box_id, shippingBox))
  // {
  //   res.success = false;
  //   return true;
  // }
  // shippingBox.currentShipment.shipmentType = req.shipment_type;
  res.success = true;
  // res.inspection_result = this->dataPtr->ariacScorer.SubmitShipment(shippingBox).total();
  res.inspection_result = 0;
  gzdbg << "Inspection result: " << res.inspection_result << std::endl;
  return true;
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleGetMaterialLocationsService(
  osrf_gear::GetMaterialLocations::Request & req,
  osrf_gear::GetMaterialLocations::Response & res)
{
  gzdbg << "Get material locations service called\n";
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto it = this->dataPtr->materialLocations.find(req.material_type);
  if (it == this->dataPtr->materialLocations.end())
  {
    gzdbg << "No known locations for material type: " << req.material_type << std::endl;
  }
  else
  {
    auto locations = it->second;
    for (auto storage_unit : locations)
    {
      osrf_gear::StorageUnit storageUnitMsg;
      storageUnitMsg.unit_id = storage_unit;
      res.storage_units.push_back(storageUnitMsg);
    }
  }
  return true;
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleAGVDeliverService(
  osrf_gear::AGVControl::Request & req, osrf_gear::AGVControl::Response & res, int agv_id)
{
  gzdbg << "AGV control service called " << agv_id << "\n";


  if (this->dataPtr->agvAnimateClient.end() == this->dataPtr->agvAnimateClient.find(agv_id))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] no animate client for agv " << agv_id);
    return false;
  }
  if (this->dataPtr->agvGetContentClient.end() == this->dataPtr->agvGetContentClient.find(agv_id))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] no content client for agv " << agv_id);
    return false;
  }

  auto & animateClient = this->dataPtr->agvAnimateClient.at(agv_id);
  auto & getContentClient = this->dataPtr->agvGetContentClient.at(agv_id);

  if (!animateClient.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] animate service does not exist for " << agv_id);
    return false;
  }
  if (!getContentClient.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] content service does not exist for " << agv_id);
    return false;
  }

  osrf_gear::DetectShipment shipment_content;
  if (!getContentClient.call(shipment_content))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to get content" << agv_id);
    return false;
  }

  std_srvs::Trigger animate;
  if (!animateClient.call(animate))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to ask agv to animate" << agv_id);
    return false;
  }

  // If AGV says it's moving, then notify scorer about shipment
  if (animate.response.success)
  {
    auto currentSimTime = this->dataPtr->world->SimTime();
    res.success = true;
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->ariacScorer.NotifyShipmentReceived(currentSimTime, req.shipment_type, shipment_content.response.shipment);
  }
  else
  {
    res.success = false;
    res.message = animate.response.message;
  }

  return true;
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::EnableConveyorBeltControl()
{
  gazebo::msgs::GzString msg;
  msg.set_data("enabled");
  this->dataPtr->conveyorEnablePub->Publish(msg);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::PopulateConveyorBelt()
{
  gzdbg << "Populate conveyor belt called.\n";
  // Publish a message on the activation_plugin of the PopulationPlugin.
  gazebo::msgs::GzString msg;
  msg.set_data("restart");
  this->dataPtr->populatePub->Publish(msg);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::AnnounceOrder(const ariac::Order & order)
{
    // Publish the order to ROS topic
    std::ostringstream logMessage;
    logMessage << "Announcing order: " << order.orderID << std::endl;
    ROS_INFO_STREAM(logMessage.str().c_str());
    gzdbg << logMessage.str() << std::endl;
    osrf_gear::Order orderMsg;
    fillOrderMsg(order, orderMsg);
    this->dataPtr->orderPub.publish(orderMsg);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::StopCurrentOrder()
{
  // Stop the current order; any previous orders that are incomplete will automatically be resumed
  if (this->dataPtr->ordersInProgress.size())
  {
    auto orderID = this->dataPtr->ordersInProgress.top().orderID;
    gzdbg << "Stopping order: " << orderID << std::endl;
    this->dataPtr->ordersInProgress.pop();
  }
}
