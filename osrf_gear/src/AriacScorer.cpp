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

#include <math.h>
#include <string>

#include <gazebo/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include "osrf_gear/AriacScorer.h"

/////////////////////////////////////////////////
AriacScorer::AriacScorer()
{
}

/////////////////////////////////////////////////
AriacScorer::~AriacScorer()
{
}

/////////////////////////////////////////////////
ariac::GameScore AriacScorer::GetGameScore()
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  return this->gameScore;
}

/////////////////////////////////////////////////
ariac::OrderScore AriacScorer::GetOrderScore(const ariac::OrderID_t & orderID)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  ariac::OrderScore score;
  auto it = this->gameScore.orderScores.find(orderID);
  if (it == this->gameScore.orderScores.end())
  {
    gzdbg << "No known order with ID: " << orderID << std::endl;
    return score;
  }
  score = it->second;
  return score;
}

/////////////////////////////////////////////////
void AriacScorer::Update(double timeStep)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);

  if (this->isProductTravelling)
  {
    this->gameScore.productTravelTime += timeStep;
  }

  if (this->newOrderReceived)
  {
    gzdbg << "New order received: " << this->newOrder.orderID << std::endl;
    this->AssignOrder(this->newOrder);
  }

  // Update the time spent on all active orders
  for (auto & item : this->gameScore.orderScores)
  {
    auto pOrderScore = &(item.second);
    if (!pOrderScore->isComplete())
    {
      pOrderScore->timeTaken += timeStep;
    }
  }

  this->newOrderReceived = false;
  this->newShippingBoxInfoReceived = false;
}

/////////////////////////////////////////////////
bool AriacScorer::IsOrderComplete(const ariac::OrderID_t & orderID)
{
  auto orderScore = GetOrderScore(orderID);
  boost::mutex::scoped_lock mutexLock(this->mutex);
  return orderScore.isComplete();
}

/////////////////////////////////////////////////
std::vector<ariac::ShippingBox> AriacScorer::GetShippingBoxes()
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  std::vector<ariac::ShippingBox> shippingBoxesVec;
  for (auto const & item : this->shippingBoxes)
  {
    shippingBoxesVec.push_back(item.second);
  }
  return shippingBoxesVec;
}

/////////////////////////////////////////////////
bool AriacScorer::GetShippingBoxById(const ariac::ShippingBoxID_t & shippingBoxID, ariac::ShippingBox & shippingBox)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  auto it = this->shippingBoxes.find(shippingBoxID);
  if (it == this->shippingBoxes.end())
  {
    gzwarn << "No known shipping box with ID: " << shippingBoxID << std::endl;
    return false;
  }
  shippingBox = it->second;
  return true;
}

/////////////////////////////////////////////////
ariac::ShipmentScore AriacScorer::SubmitShipment(const ariac::ShippingBox & shippingBox)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  ariac::ShipmentScore shipmentScore;
  ariac::ShipmentType_t shipmentType = shippingBox.currentShipment.shipmentType;

  // Determine order and shipment the shipping box is from
  ariac::Order relevantOrder;
  ariac::Shipment assignedShipment;

  for (const auto & order : this->ordersInProgress)
  {
    auto it = find_if(order.shipments.begin(), order.shipments.end(),
      [&shipmentType](const ariac::Shipment& shipment) {
        return shipment.shipmentType == shipmentType;
      });
    if (it != order.shipments.end())
    {
      assignedShipment = *it;
      relevantOrder = order;
      break;
    }
  }

  // Ignore unknown shipping boxes
  ariac::OrderID_t orderId = relevantOrder.orderID;
  if (orderId == "")
  {
    gzdbg << "No known shipment type: " << shipmentType << std::endl;
    gzdbg << "Known shipment types are: " << std::endl;
    for (const ariac::Order & order : this->ordersInProgress)
    {
      for (const ariac::Shipment & shipment : order.shipments)
      {
        gzdbg << shipment.shipmentType << std::endl;
      }
    }
    return shipmentScore;
  }

  // Do not allow re-submission of shipping boxes - just return the existing score.
  auto relevantOrderScore = &this->gameScore.orderScores[orderId];
  auto it = relevantOrderScore->shipmentScores.find(shipmentType);
  if (it != relevantOrderScore->shipmentScores.end())
  {
    shipmentScore = it->second;
    if (shipmentScore.isSubmitted)
    {
      gzdbg << "Shipment already submitted, not rescoring: " << shipmentType << std::endl;
      return shipmentScore;
    }
  }

  // Evaluate the shipping box against the shipment it contains
  shipmentScore = ScoreShippingBox(shippingBox, assignedShipment);

  // Mark the shipment as submitted
  shipmentScore.isSubmitted = true;

  gzdbg << "Score from shipment '" << shippingBox.shippingBoxID << "': " << shipmentScore.total() << std::endl;

  // Add the shipment to the game score
  relevantOrderScore->shipmentScores[shipmentType] = shipmentScore;

  return shipmentScore;
}

/////////////////////////////////////////////////
ariac::ShipmentScore AriacScorer::ScoreShippingBox(const ariac::ShippingBox & shippingBox, const ariac::Shipment & assignedShipment)
{
  ariac::Shipment shipment = shippingBox.currentShipment;
  ariac::ShipmentType_t shipmentType = shippingBox.currentShipment.shipmentType;
  ariac::ShipmentScore score;
  score.shipmentType = shipmentType;
  gzdbg << "Scoring shipment: " << shipment << std::endl;

  auto numAssignedProducts = assignedShipment.products.size();
  auto numCurrentProducts = shipment.products.size();
  gzdbg << "Comparing the " << numAssignedProducts << " assigned products with the current " << \
    numCurrentProducts << " products" << std::endl;

  // Count the number of each type of assigned product
  std::map<std::string, unsigned int> assignedProductTypeCount, currentProductTypeCount;
  for (const auto & obj : assignedShipment.products)
  {
    if (assignedProductTypeCount.find(obj.type) == assignedProductTypeCount.end())
    {
      assignedProductTypeCount[obj.type] = 0;
    }
    assignedProductTypeCount[obj.type] += 1;
  }

  gzdbg << "Checking product counts" << std::endl;

  bool assignedProductsMissing = false;
  for (auto & value : assignedProductTypeCount)
  {
    auto assignedProductType = value.first;
    auto assignedProductCount = value.second;
    auto currentProductCount =
      std::count_if(shipment.products.begin(), shipment.products.end(),
        [assignedProductType](ariac::Product k) {return !k.isFaulty && k.type == assignedProductType;});
    gzdbg << "Found " << currentProductCount << \
      " products of type '" << assignedProductType << "'" << std::endl;
    score.productPresence +=
      std::min(long(assignedProductCount), currentProductCount) * scoringParameters.productPresence;
    if (currentProductCount < assignedProductCount)
    {
      assignedProductsMissing = true;
    }
  }
  if (!assignedProductsMissing && numCurrentProducts == numAssignedProducts)
  {
    gzdbg << "All products in shipment and no extra products detected." << std::endl;
    score.allProductsBonus += scoringParameters.allProductsBonusFactor * numAssignedProducts;
  }

  gzdbg << "Checking product poses" << std::endl;
  // Keep track of which assigned products have already been 'matched' to one on the shippingBox.
  // This is to prevent multiple products being close to a single target pose both scoring points.
  std::vector<ariac::Product> remainingAssignedProducts(assignedShipment.products);

  for (const auto & currentProduct : shipment.products)
  {
    for (auto it = remainingAssignedProducts.begin(); it != remainingAssignedProducts.end(); ++it)
    {
      // Only check poses of products of the same type
      auto assignedProduct = *it;
      if (assignedProduct.type != currentProduct.type)
        continue;

      gzdbg << "\n\nEvaluating product of type '" << currentProduct.type << "'" << std::endl;

      // Ignore faulty products
      if (currentProduct.isFaulty)
      {
        gzdbg << "Skipping product because it is faulty" << std::endl;
        continue;
      }

      // Check the position of the product (ignoring orientation)
      gzdbg << "Comparing pose '" << currentProduct.pose << \
        "' with the assigned pose '" << assignedProduct.pose << "'" << std::endl;
      ignition::math::Vector3d posnDiff(
        currentProduct.pose.Pos().X() - assignedProduct.pose.Pos().X(),
        currentProduct.pose.Pos().Y() - assignedProduct.pose.Pos().Y(),
        0);
      gzdbg << "Position error: " << posnDiff.Length() << std::endl;
      if (posnDiff.Length() > scoringParameters.distanceThresh)
      {
        gzdbg << "Skipping product because it is not in the correct position" << std::endl;
        continue;
      }
      gzdbg << "Product of type '" << currentProduct.type << \
        "' in the correct position" << std::endl;
      score.productPose += scoringParameters.productPosition;

      // Check the orientation of the product.
      ignition::math::Quaterniond objOrientation = currentProduct.pose.Rot();
      ignition::math::Quaterniond orderOrientation = assignedProduct.pose.Rot();

      // Filter products that aren't in the appropriate orientation (loosely).
      // If the quaternions represent the same orientation, q1 = +-q2 => q1.dot(q2) = +-1
      double orientationDiff = objOrientation.Dot(orderOrientation);
      // TODO: this value can probably be derived using relationships between
      // euler angles and quaternions.
      double quaternionDiffThresh = 0.05;
      gzdbg << "Cosine of angle between orientations (quaternion dot product): " << \
        orientationDiff << std::endl;
      if (std::abs(orientationDiff) < (1.0 - quaternionDiffThresh))
      {
        gzdbg << "Skipping product because it is not in the correct orientation (roughly)" << std::endl;
        continue;
      }

      // Filter the yaw based on a threshold set in radians (more user-friendly).
      double angleDiff = objOrientation.Yaw() - orderOrientation.Yaw();
      gzdbg << "Orientation error (yaw): " << std::abs(angleDiff) << \
        " (or " << std::abs(std::abs(angleDiff) - 2 * M_PI) << ")" << std::endl;
      if (std::abs(angleDiff) > scoringParameters.orientationThresh)
      {
        // Account for wrapping in angles. E.g. -pi compared with pi should "pass".
        if (std::abs(std::abs(angleDiff) - 2 * M_PI) > scoringParameters.orientationThresh)
        {
          gzdbg << "Skipping product because it is not in the correct orientation" << std::endl;
          continue;
        }
      }

      gzdbg << "Product of type '" << currentProduct.type << \
        "' in the correct orientation" << std::endl;
      score.productPose += scoringParameters.productOrientation;

      // Once a match is found, don't permit it to be matched again
      remainingAssignedProducts.erase(it);
      break;
    }
  }

  // Check if all assigned products have been matched to one in the shipping box
  if (remainingAssignedProducts.empty())
  {
    score.isComplete = true;
  }

  return score;
}

/////////////////////////////////////////////////
void AriacScorer::OnShippingBoxInfoReceived(const osrf_gear::ShippingBoxContents::ConstPtr & shippingBoxMsg)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);

  // Get the ID of the shipping box that the message is from.
  std::string shippingBoxID = shippingBoxMsg->shipping_box;

  if (this->shippingBoxes.find(shippingBoxID) == this->shippingBoxes.end())
  {
    // This is the first time we've heard from this shipping box: initialize it.
    this->shippingBoxes[shippingBoxID] = ariac::ShippingBox(shippingBoxID);
  }

  // Update the state of the shippingBox.
  // TODO: this should be moved outside of the callback
  // Do this even if the shipment isn't product of the current order because maybe it
  // will be product of future orders.
  this->newShippingBoxInfoReceived = true;
  ariac::Shipment shipmentState;
  FillShipmentFromMsg(shippingBoxMsg, shipmentState);
  this->shippingBoxes[shippingBoxID].UpdateShipmentState(shipmentState);
}

/////////////////////////////////////////////////
void AriacScorer::OnOrderReceived(const osrf_gear::Order::ConstPtr & orderMsg)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  gzdbg << "Received an order" << std::endl;
  this->newOrderReceived = true;

  ariac::Order order;
  order.orderID = orderMsg->order_id;
  // Initialize the name of each of the expected shipments.
  for (const auto & shipmentMsg : orderMsg->shipments)
  {
    ariac::ShipmentType_t shipmentType = shipmentMsg.shipment_type;
    ariac::Shipment assignedShipment;
    FillShipmentFromMsg(shipmentMsg, assignedShipment);
    order.shipments.push_back(assignedShipment);
  }
  this->newOrder = order;
}

/////////////////////////////////////////////////
void AriacScorer::AssignOrder(const ariac::Order & order)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  gzdbg << "Assigned order: " << order << std::endl;
  ariac::OrderID_t orderID = order.orderID;
  if (this->gameScore.orderScores.find(orderID) == this->gameScore.orderScores.end())
  {
    // This is a previously unseen order: start scoring from scratch
    auto orderScore = ariac::OrderScore();
    orderScore.orderID = orderID;
    for (auto const & shipment : order.shipments)
    {
      auto shipmentScore = ariac::ShipmentScore();
      shipmentScore.shipmentType = shipment.shipmentType;
      orderScore.shipmentScores[shipment.shipmentType] = shipmentScore;
    }
    this->gameScore.orderScores[orderID] = orderScore;
  }

  this->ordersInProgress.push_back(order);
}

/////////////////////////////////////////////////
void AriacScorer::UpdateOrder(const ariac::Order & order)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  gzdbg << "Updated order: " << order << std::endl;
  ariac::OrderID_t orderID = order.orderID;
  auto it = find_if(this->ordersInProgress.begin(), this->ordersInProgress.end(),
      [&orderID](const ariac::Order& o) {
        return o.orderID == orderID;
      });
  if (it == this->ordersInProgress.end())
  {
    gzerr << "No order with ID: " << orderID << std::endl;
    return;
  }
  // TODO: re-evaluating any existing score.
  *it = order;
}

/////////////////////////////////////////////////
ariac::OrderScore AriacScorer::UnassignOrder(const ariac::OrderID_t & orderID)
{
  gzdbg << "Unassign order request for: " << orderID << std::endl;
  ariac::OrderScore orderScore;
  boost::mutex::scoped_lock mutexLock(this->mutex);
  auto it1 = find_if(this->ordersInProgress.begin(), this->ordersInProgress.end(),
      [&orderID](const ariac::Order& o) {
        return o.orderID == orderID;
      });
  if (it1 == this->ordersInProgress.end())
  {
    gzerr << "No order with ID: " << orderID << std::endl;
    return orderScore;
  }
  auto it = this->gameScore.orderScores.find(orderID);
  if (it == this->gameScore.orderScores.end())
  {
    gzerr << "No order score with ID: " << orderID << std::endl;
    return orderScore;
  }
  orderScore = it->second;
  gzdbg << "Unassigning order: " << orderID << std::endl;
  this->ordersInProgress.pop_back();
  return orderScore;
}

/////////////////////////////////////////////////
void AriacScorer::FillShipmentFromMsg(const osrf_gear::ShippingBoxContents::ConstPtr &shippingBoxMsg, ariac::Shipment &shipment)
{
  shipment.products.clear();
  for (const auto & objMsg : shippingBoxMsg->products)
  {
    ariac::Product obj;
    obj.type = ariac::DetermineModelType(objMsg.type);
    obj.isFaulty = objMsg.is_faulty;
    geometry_msgs::Point p = objMsg.pose.position;
    geometry_msgs::Quaternion o = objMsg.pose.orientation;
    ignition::math::Vector3d objPosition(p.x, p.y, p.z);
    ignition::math::Quaterniond objOrientation(o.w, o.x, o.y, o.z);
    objOrientation.Normalize();
    obj.pose = ignition::math::Pose3d(objPosition, objOrientation);
    shipment.products.push_back(obj);
  }
}

/////////////////////////////////////////////////
void AriacScorer::FillShipmentFromMsg(const osrf_gear::Shipment &shipmentMsg, ariac::Shipment &shipment)
{
  shipment.products.clear();
  for (const auto & objMsg : shipmentMsg.products)
  {
    ariac::Product obj;
    obj.type = ariac::DetermineModelType(objMsg.type);
    geometry_msgs::Point p = objMsg.pose.position;
    geometry_msgs::Quaternion o = objMsg.pose.orientation;
    ignition::math::Vector3d objPosition(p.x, p.y, p.z);
    ignition::math::Quaterniond objOrientation(o.w, o.x, o.y, o.z);
    obj.pose = ignition::math::Pose3d(objPosition, objOrientation);
    shipment.products.push_back(obj);
  }
}

/////////////////////////////////////////////////
void AriacScorer::OnGripperStateReceived(const osrf_gear::VacuumGripperState &stateMsg)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  this->isProductTravelling = stateMsg.enabled && stateMsg.attached;
}
