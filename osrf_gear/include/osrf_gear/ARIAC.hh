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
#ifndef _ARIAC_HH_
#define _ARIAC_HH_

#include <ostream>
#include <map>
#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <ignition/math/Pose3.hh>

namespace ariac
{
  using namespace gazebo;

  typedef std::string KitType_t;
  typedef std::string ShipmentType_t;
  typedef std::string ShippingBoxID_t;
  typedef std::string OrderID_t;

  /// \brief The score of a shipment.
  class ShipmentScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj ShipmentScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const ShipmentScore &_obj)
    {
      _out << "<shipment_score " << _obj.shipmentType << ">" << std::endl;
      _out << "Completion score: [" << _obj.total() << "]" << std::endl;
      _out << "Complete: [" << (_obj.isComplete ? "true" : "false") << "]" << std::endl;
      _out << "Submitted: [" << (_obj.isSubmitted ? "true" : "false") << "]" << std::endl;
      _out << "Product presence score: [" << _obj.productPresence << "]" << std::endl;
      _out << "All products bonus: [" << _obj.allProductsBonus << "]" << std::endl;
      _out << "Product pose score: [" << _obj.productPose << "]" << std::endl;
      _out << "</shipment_score>" << std::endl;
      return _out;
    }
    public: ShipmentType_t shipmentType;
            double productPresence = 0.0;
            double allProductsBonus = 0.0;
            double productPose = 0.0;
            bool isComplete = false;  // All products present
            bool isSubmitted = false;  // the shipment has been submitted for evaluation

            /// \brief Calculate the total score.
            double total() const
            {
              return productPresence + allProductsBonus + productPose;
            }
  };

  /// \brief The score of an order.
  class OrderScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj OrderScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const OrderScore &_obj)
    {
      _out << "<order_score " << _obj.orderID << ">" << std::endl;
      _out << "Total order score: [" << _obj.total() << "]" << std::endl;
      _out << "Time taken: [" << _obj.timeTaken << "]" << std::endl;
      _out << "Complete: [" << (_obj.isComplete() ? "true" : "false") << "]" << std::endl;
      for (const auto & item : _obj.shipmentScores)
      {
        _out << item.second << std::endl;
      }
      _out << "</order_score>" << std::endl;
      return _out;
    }

    /// \brief Mapping between shipment IDs and scores.
    public: std::map<ShipmentType_t, ShipmentScore> shipmentScores;

            /// \brief ID of the order being scored.
            OrderID_t orderID;

            /// \brief Time in seconds spend on the order.
            double timeTaken = 0.0;

            /// \brief Calculate if the order is complete.
            /// \return True if all shipping boxes have been submitted.
            ///   Will return false if there are no shipping boxes in the order.
            bool isComplete() const
            {
              bool isOrderComplete = !this->shipmentScores.empty();
              for (const auto & item : this->shipmentScores)
              {
                isOrderComplete &= item.second.isSubmitted;
                if (!isOrderComplete)
                {
                  break;
                }
              }
              return isOrderComplete;
            };

            /// \brief Calculate the total score.
            double total() const
            {
              double total = 0.0;
              for (const auto & item : this->shipmentScores)
              {
                total += item.second.total();
              }
              return total;
            };
  };

  /// \brief The score of a competition run.
  class GameScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj GameScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const GameScore &_obj)
    {
      _out << "<game_score>" << std::endl;
      _out << "Total game score: [" << _obj.total() << "]" << std::endl;
      _out << "Total process time: [" << _obj.totalProcessTime << "]" << std::endl;
      _out << "Product travel time: [" << _obj.productTravelTime << "]" << std::endl;
      for (const auto & item : _obj.orderScores)
      {
        _out << item.second << std::endl;
      }
      _out << "</game_score>" << std::endl;
      return _out;
    }

    public: double totalProcessTime = 0.0;
            double productTravelTime = 0.0;
            double planningTime = 0.0;
            double productTravelDistance = 0.0;
            double manipulatorTravelDistance = 0.0;

            // The score of each of the orders during the game.
            std::map<OrderID_t, OrderScore> orderScores;

            /// \brief Calculate the total score.
            double total() const
            {
              double total = 0;
              /*
              total += totalProcessTime;
              total += productTravelTime;
              total += planningTime;
              total += productTravelDistance;
              total += manipulatorTravelDistance;
              */

              for (const auto & item : this->orderScores)
              {
                total += item.second.total();
              }
              return total;
            };
  };

  /// \brief The parameters used for scoring the competition.
  // TODO: this should have a different data type
  class ScoringParameters
  {
    /// \brief Equality comparison operator.
    /// \param[in] sp1 First parameters to compare.
    /// \param[in] sp2 Second parameters to compare.
    /// \return True if sp1 == sp2.
    public: friend bool operator==(const ScoringParameters &sp1, const ScoringParameters &sp2)
    {
      return (
        sp1.productPresence == sp2.productPresence &&
        sp1.productPosition == sp2.productPosition &&
        sp1.productOrientation == sp2.productOrientation &&
        sp1.allProductsBonusFactor == sp2.allProductsBonusFactor &&
        sp1.distanceThresh == sp2.distanceThresh);
    }

    /// \brief Inequality comparison operator.
    /// \param[in] sp1 First parameters to compare.
    /// \param[in] sp2 Second parameters to compare.
    /// \return True if sp1 != sp2.
    public: friend bool operator!=(const ScoringParameters &sp1, const ScoringParameters &sp2)
    {
      return !(sp1 == sp2);
    }

    public: double productPresence = 1.0;
    public: double productPosition = 0.0;
    public: double productOrientation = 1.0;

    // Bonus when all products in the shipment: factor * (number of products)
    public: double allProductsBonusFactor = 1.0;

    // Acceptable distance in meters to product's target position.
    // The measured distance is between the center of the model and its target,
    // projected onto the shipping box.
    public: double distanceThresh = 0.03;

    // Acceptable difference in radians to product's target orientation.
    // The measured difference is from a top-down view of the shipping box, but only if
    // the quaternions are aligned.
    public: double orientationThresh = 0.1;
  };

  /// \brief Determine the model name without namespace
  std::string TrimNamespace(const std::string &modelName)
  {
    // Trim namespaces
    size_t index = modelName.find_last_of('|');
    return modelName.substr(index + 1);
  }

  /// \brief Determine the type of a gazebo model from its name
  std::string DetermineModelType(const std::string &modelName)
  {
    std::string modelType(TrimNamespace(modelName));

    // Trim trailing underscore and number caused by inserting multiple of the same model
    size_t index = modelType.find_last_not_of("0123456789");
    if (modelType[index] == '_' && index > 1)
    {
      modelType = modelType.substr(0, index);
    }

    // Trim "_clone" suffix if exists
    index = modelType.rfind("_clone");
    if (index != std::string::npos)
    {
      modelType.erase(index);
    }

    return modelType;
  }

  /// \brief Determine the ID of a gazebo model from its name
  std::string DetermineModelId(const std::string &modelName)
  {
    std::string modelId(TrimNamespace(modelName));

    // Trim trailing underscore and number caused by inserting multiple of the same model
    size_t index = modelId.find_last_not_of("0123456789");
    if (modelId[index] == '_' && index > 1)
    {
      modelId = modelId.substr(index + 1);
    }

    return modelId;
  }

  /// \brief Class to store information about each product contained in a shipment.
  class Product
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj Shipment object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Product &_obj)
    {
      _out << "<product>" << std::endl;
      _out << "Type: [" << _obj.type << "]" << std::endl;
      _out << "Faulty: [" << (_obj.isFaulty ? "true" : "false") << "]" << std::endl;
      _out << "Pose: [" << _obj.pose << "]" << std::endl;
      _out << "</product>" << std::endl;
      return _out;
    }

    /// \brief Product type.
    public: std::string type;

    /// \brief Whether or not the product is faulty.
    public: bool isFaulty;

    /// \brief Pose in which the product should be placed.
    public: ignition::math::Pose3d pose;

  };

  /// \brief Class to store information about a shipment.
  class Shipment
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _shipment shipment to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Shipment &_shipment)
    {
      _out << "<shipment type='" << _shipment.shipmentType << "'>";
      for (const auto & obj : _shipment.products)
        _out << std::endl << obj;
      _out << std::endl << "</shipment>" << std::endl;

      return _out;
    }

    /// \brief The type of the shipment.
    public: ShipmentType_t shipmentType;

    /// \brief A shipment is composed of multiple products.
    public: std::vector<Product> products;
  };

  /// \brief Class to store information about each object contained in a kit.
  class KitObject
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj Kit object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const KitObject &_obj)
    {
      _out << "<object>" << std::endl;
      _out << "Type: [" << _obj.type << "]" << std::endl;
      _out << "Faulty: [" << (_obj.isFaulty ? "true" : "false") << "]" << std::endl;
      _out << "Pose: [" << _obj.pose << "]" << std::endl;
      _out << "</object>" << std::endl;
      return _out;
    }

    /// \brief Object type.
    public: std::string type;

    /// \brief Whether or not the object is faulty.
    public: bool isFaulty;

    /// \brief Pose in which the object should be placed.
    public: ignition::math::Pose3d pose;

  };

  /// \brief Class to store information about a kit.
  class Kit
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _kit kit to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Kit &_kit)
    {
      _out << "<kit type='" << _kit.kitType << "'>";
      for (const auto & obj : _kit.objects)
        _out << std::endl << obj;
      _out << std::endl << "</kit>" << std::endl;

      return _out;
    }

    /// \brief The type of the kit.
    public: KitType_t kitType;

    /// \brief A kit is composed of multiple objects.
    public: std::vector<KitObject> objects;
  };

  /// \brief Class to store information about an order.
  class Order
  {
    /// \brief Less than operator.
    /// \param[in] _order Other order to compare.
    /// \return True if this < _order.
    public: bool operator<(const Order &_order) const
    {
      return this->startTime < _order.startTime;
    }

    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _order Order to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Order &_order)
    {
      _out << "<Order>" << std::endl;
      _out << "Start time: [" << _order.startTime << "]" << std::endl;
      _out << "Shipments:" << std::endl;
      for (const auto & item : _order.shipments)
      {
        _out << item << std::endl;
      }
      _out << "</order>" << std::endl;

      return _out;
    }

    /// \brief The ID of this order.
    public: OrderID_t orderID;

    /// \brief Simulation time in which the order should be triggered.
    public: double startTime;

    /// \brief After how many unwanted products to interrupt the previous order (-1 for never).
    public: int interruptOnUnwantedProducts;

    /// \brief After how many wanted products to interrupt the previous order (-1 for never).
    public: int interruptOnWantedProducts;

    /// \brief Simulation time in seconds permitted for the order to be
    /// completed before cancelling it. Infinite by default.
    public: double allowedTime;

    /// \brief An order is composed of multiple shipments of different types.
    public: std::vector<Shipment> shipments;

    /// \brief Simulation time in seconds spent on this order.
    public: double timeTaken;
  };
}
#endif
