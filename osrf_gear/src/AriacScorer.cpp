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
void AriacScorer::NotifyOrderStarted(gazebo::common::Time time, const osrf_gear::Order & order)
{
  AriacScorer::OrderInfo orderInfo;
  orderInfo.start_time = time;
  orderInfo.order = osrf_gear::Order::ConstPtr(new osrf_gear::Order(order));

  boost::mutex::scoped_lock mutexLock(this->mutex);

  orderInfo.priority = 1;
  if (!this->orders.empty())
  {
    // orders after the first are implicitly higher priority
    orderInfo.priority = 3;
  }

  auto it = this->orders.find(order.order_id);
  if (it != this->orders.end())
  {
    gzerr << "[ARIAC ERROR] Order with duplicate ID '" << order.order_id << "'; overwriting\n";
  }

  this->orders[order.order_id] = orderInfo;
}

/////////////////////////////////////////////////
void AriacScorer::NotifyOrderUpdated(gazebo::common::Time time, ariac::OrderID_t old_order, const osrf_gear::Order & order)
{
  AriacScorer::OrderUpdateInfo updateInfo;
  updateInfo.update_time = time;
  updateInfo.original_order_id = old_order; 
  updateInfo.order = osrf_gear::Order::ConstPtr(new osrf_gear::Order(order));

  boost::mutex::scoped_lock mutexLock(this->mutex);
  auto it = this->orders.find(order.order_id);
  if (it != this->orders.end())
  {
    gzerr << "[ARIAC ERROR] Asked to update nonexistant order '" << order.order_id << "'; ignoring\n";
    return;
  }

  this->order_updates.push_back(updateInfo);
}

/////////////////////////////////////////////////
void AriacScorer::NotifyShipmentReceived(gazebo::common::Time time, ariac::ShipmentType_t type, const osrf_gear::DetectedShipment & shipment)
{
  AriacScorer::ShipmentInfo shipmentInfo;
  shipmentInfo.submit_time = time;
  shipmentInfo.type = type;
  shipmentInfo.shipment = osrf_gear::DetectedShipment::ConstPtr(new osrf_gear::DetectedShipment(shipment));

  boost::mutex::scoped_lock mutexLock(this->mutex);
  this->shipments.push_back(shipmentInfo);
}

/////////////////////////////////////////////////
void AriacScorer::NotifyArmArmCollision(gazebo::common::Time /*time*/)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  this->arm_arm_collision = true;
}

/////////////////////////////////////////////////
ariac::GameScore AriacScorer::GetGameScore()
{
  boost::mutex::scoped_lock mutexLock(this->mutex);

  ariac::GameScore game_score;

  // arm/arm collision results in zero score, but keep going for logging
  game_score.was_arm_arm_collision = this->arm_arm_collision;

  // Calculate the current score based on received orders and shipments
  // For each order, how many shipments was it supposed to have?
  // Set up shipment sc
  for (auto & opair : this->orders)
  {
    auto order_id = opair.first;
    auto order_info = opair.second;

    gazebo::common::Time start_time = order_info.start_time;
    int priority = order_info.priority;
    osrf_gear::Order::ConstPtr order = order_info.order;

    // If order was updated, score based on the lastest version of it
    for (auto & update_info : this->order_updates)
    {
      if (update_info.original_order_id == order_id)
      {
        order = update_info.order;
        start_time = update_info.update_time;
      }
    }

    // Create score class for order
    ariac::OrderScore order_score;
    order_score.orderID = order_id;
    order_score.priority = priority;
    auto oit = game_score.orderScores.find(order_id);
    if (oit != game_score.orderScores.end())
    {
      gzerr << "[ARIAC ERROR] Multiple orders of duplicate ids:" << order_score.orderID << "\n";
    }

    // Create score classes for shipments
    for (const auto & expected_shipment : order->shipments)
    {
      ariac::ShipmentScore shipment_score;
      shipment_score.shipmentType = expected_shipment.shipment_type;
      auto it = order_score.shipmentScores.find(expected_shipment.shipment_type);
      if (it != order_score.shipmentScores.end())
      {
        gzerr << "[ARIAC ERROR] Order contained duplicate shipment types:" << expected_shipment.shipment_type << "\n";
      }
      order_score.shipmentScores[expected_shipment.shipment_type] = shipment_score;
    }

    // Find actual shipments that belong to this order
    for (const auto & desired_shipment : order->shipments)
    {
      for (const auto & shipment_info : this->shipments)
      {
        if (desired_shipment.shipment_type == shipment_info.type)
        {
          if (shipment_info.submit_time < start_time)
          {
            // Maybe order was updated, this shipment was submitted too early
            continue;
          }
          // Found actual shipment, score it
          auto & scorer = order_score.shipmentScores[desired_shipment.shipment_type];
          scorer.isSubmitted = true;
          scorer.submit_time = shipment_info.submit_time;

          // Separate faulty and non-faulty products
          bool has_faulty_product = false;
          std::vector<osrf_gear::DetectedProduct> non_faulty_products;
          for (const auto & actual_product : shipment_info.shipment->products)
          {
            if (actual_product.is_faulty)
            {
              has_faulty_product = true;
            }
            else
            {
              non_faulty_products.push_back(actual_product);
            }
          }

          // Copy desired to ensure only one is matched with an actual product
          std::vector<osrf_gear::Product> unclaimed_desired_products(desired_shipment.products);

          bool has_unwanted_product = false;
          // Match products with desired products
          std::vector<std::pair<osrf_gear::Product, osrf_gear::DetectedProduct>> matched_products;
          for (const auto & actual_product : non_faulty_products)
          {
            auto best_match = unclaimed_desired_products.end();
            double closest_distance = std::numeric_limits<double>::max();
            for (auto desired_it = unclaimed_desired_products.begin(); desired_it != unclaimed_desired_products.end(); ++desired_it)
            {
              // Only match if they have the same type
              if (desired_it->type != actual_product.type)
              {
                continue;
              }
              ignition::math::Vector3d posnDiff(
                desired_it->pose.position.x - actual_product.pose.position.x,
                desired_it->pose.position.y - actual_product.pose.position.y,
                0);
              double distance = posnDiff.Length();

              if (distance < closest_distance)
              {
                best_match = desired_it;
                closest_distance = distance;
              }
            }
            if (best_match != unclaimed_desired_products.end())
            {
              // Erase desired product once claimed so it's not matched again
              matched_products.push_back(std::make_pair(*best_match, actual_product));
              unclaimed_desired_products.erase(best_match);
            }
            else
            {
              has_unwanted_product = true;
            }
          }

          // Figure out the completion score
          // One point for each correct product in the shipment
          scorer.productPresence = matched_products.size();
          // Bonus points if all desired products are present and no undesired are present
          if (matched_products.size() == desired_shipment.products.size())
          {
            // All desired products were present
            scorer.isComplete = true;

            // give a bonus if no additional products were present
            if (matched_products.size() == shipment_info.shipment->products.size()
            && !has_unwanted_product && !has_faulty_product)
            {
              scorer.allProductsBonus = matched_products.size();
            }
          }
          scorer.productPose = 0.0;
          // Add points for each product in the correct pose
          const double translation_target = 0.03;  // 3 cm
          const double orientation_target = 0.1;  // 0.1 rad
          for (auto & ppair : matched_products)
          {
            // get translation distance
            ignition::math::Vector3d posnDiff(
              ppair.first.pose.position.x - ppair.second.pose.position.x,
              ppair.first.pose.position.y - ppair.second.pose.position.y,
              0);
            const double distance = posnDiff.Length();
            if (distance > translation_target)
            {
              // Skipping product because translation error is too big
              continue;
            }

            ignition::math::Quaterniond orderOrientation(
              ppair.first.pose.orientation.w,
              ppair.first.pose.orientation.x,
              ppair.first.pose.orientation.y,
              ppair.first.pose.orientation.z);
            ignition::math::Quaterniond objOrientation(
              ppair.second.pose.orientation.w,
              ppair.second.pose.orientation.x,
              ppair.second.pose.orientation.y,
              ppair.second.pose.orientation.z);

            // Filter products that aren't in the appropriate orientation (loosely).
            // If the quaternions represent the same orientation, q1 = +-q2 => q1.dot(q2) = +-1
            const double orientationDiff = objOrientation.Dot(orderOrientation);
            // TODO: this value can probably be derived using relationships between
            // euler angles and quaternions.
            const double quaternionDiffThresh = 0.05;
            if (std::abs(orientationDiff) < (1.0 - quaternionDiffThresh))
            {
              // Skipping product because it is not in the correct orientation (roughly)
              continue;
            }

            // Filter the yaw based on a threshold set in radians (more user-friendly).
            // Account for wrapping in angles. E.g. -pi compared with pi should "pass".
            double angleDiff = objOrientation.Yaw() - orderOrientation.Yaw();
            if ( (std::abs(angleDiff) < orientation_target)
              || (std::abs(std::abs(angleDiff) - 2 * M_PI) <= orientation_target))
            {
               scorer.productPose += 1.0;
            }
          }
        }
      }
    }

    // Figure out the time taken to complete an order
    if (order_score.isComplete())
    {
      // The latest submitted shipment time is the order completion time
      gazebo::common::Time end = start_time;
      for (auto & sspair : order_score.shipmentScores)
      {
        if (sspair.second.submit_time > end)
        {
          end = sspair.second.submit_time;
        }
      }
      order_score.timeTaken = (end - start_time).Double();
    }

    game_score.orderScores[order_id] = order_score;
  }

  return game_score;
}
