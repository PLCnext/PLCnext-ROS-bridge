#ifndef BRIDGE_TYPE_H
#define BRIDGE_TYPE_H

#include "phoenix_bridge/include_types.h"
#include "phoenix_bridge/phoenix_comm.h"

#include <vector>
#include <chrono>
#include <functional>

/**
 * @brief Class to handle creating a bridge per supported ROS msg type
 * We create one BridgeType for odom, one for twist, etc...
 * A given bridge type can have several "ports" under it, where a port is a publisher or a subscriber
 * This way we can have several pub/subs for odom, several pub/subs for twist etc..
 * The number of ports is derived from config/test_params.yaml
 */
template <typename T>
class BridgeType : public rclcpp::Node
{
public:
  BridgeType(std::string node_name);
  void init();

private:
  PhoenixComm<T> comm_;
  std::vector<std::string> pub_topics_, pub_datapaths_;
  std::vector<std::string> sub_topics_, sub_datapaths_;
  std::vector<long int> pub_freqs_, sub_freqs_;
  std::vector<rclcpp::TimerBase::SharedPtr> pub_timers_;
  std::vector<typename rclcpp::Publisher<T>::SharedPtr> pubs_;
  std::vector<typename rclcpp::Subscription<T>::SharedPtr> subs_;

  void getPortParams();
  void spawnPublishers();
  void spawnSubscribers();

};

template<typename T> inline
BridgeType<T>::BridgeType(std::string node_name) :
  Node(node_name)
{
  init();
}

/**
 * @brief Read the rosparams to create ports and spawn pubs & subs
 * @param param_name The simple name for the parameters of this type ex: "odometry" or "twist"
 */
template<typename T> inline
void BridgeType<T>::init()
{

  this->getPortParams();

  /// Initialise communication layer
  RCLCPP_INFO_STREAM(this->get_logger(),"Initialising gRCP channel @" << this->get_parameter("grpc.address").as_string());
  comm_.init(this->get_parameter("grpc.address").as_string());

  /// Spawn pubs
  this->spawnPublishers();

  /// Spawn subs
  this->spawnSubscribers();
}

/**
 * @brief Read params for this node. Param names are hardcoded so the param file should have a very specific structure.
 * @param param_name The fully resolved param path ex: "/odometry/publishers"
 * @param port_vector Save all the read param data into a vector of PortParams
 */
template<typename T> inline
void BridgeType<T>::getPortParams()
{
  /// @todo Try catch everything
  this->declare_parameter("grpc.address");
  this->declare_parameter("grpc.type");
  this->declare_parameter("msg_type");
  this->declare_parameter("publishers.topics");
  this->declare_parameter("publishers.datapaths");
  this->declare_parameter("publishers.frequencies");
  this->declare_parameter("subscribers.topics");
  this->declare_parameter("subscribers.datapaths");
  this->declare_parameter("subscribers.frequencies");

  pub_topics_ = this->get_parameter("publishers.topics").as_string_array();
  pub_datapaths_ = this->get_parameter("publishers.datapaths").as_string_array();
  pub_freqs_ = this->get_parameter("publishers.frequencies").as_integer_array();

  sub_topics_ = this->get_parameter("subscribers.topics").as_string_array();
  sub_datapaths_ = this->get_parameter("subscribers.datapaths").as_string_array();
  sub_freqs_ = this->get_parameter("subscribers.frequencies").as_integer_array();

  if (!((pub_topics_.size() == pub_datapaths_.size())
         && (pub_topics_.size() == pub_freqs_.size())))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Publisher array sizes not equal!!");
    rclcpp::shutdown();
  }
  if (!((sub_topics_.size() == sub_datapaths_.size())
         && (sub_topics_.size() == sub_freqs_.size())))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Subscriber array sizes not equal!!");
    rclcpp::shutdown();
  }
}

/**
* @brief Spawn publishers based on the read rosparams for this type
*/
template<typename T> inline
void BridgeType<T>::spawnPublishers()
{
  for (size_t i=0; i<pub_topics_.size(); i++)
  {
    RCLCPP_INFO(this->get_logger(),
                       "Spawning publisher  [%s, %s, %d]",
                          pub_topics_[i].c_str(), pub_datapaths_[i].c_str(), pub_freqs_[i]);

    /// The publishers created here are stored in memory so that even when the timer callback goes out of scope,
    /// the publisher is held. This consumes more memory
    /// If instead the pub is created only in the timer callback, especially for low frequency publishers,
    /// it would appear in between timer callbacks that there is no publisher. This consumers less memory.
    typename rclcpp::Publisher<T>::SharedPtr pub = this->create_publisher<T>(pub_topics_[i], 1000);
    pubs_.push_back(pub);
    pub_timers_.push_back(
          this->create_wall_timer(std::chrono::milliseconds(1000/pub_freqs_[i]),
                                  [this, i, pub]()-> void{
                                      T msg_rcvd;
                                      comm_.getFromPLC(pub_datapaths_[i], msg_rcvd);
                                      typename T::SharedPtr msg_ptr = std::make_shared<T>(msg_rcvd);
                                      pub->publish(msg_rcvd); /// @todo: is this 0 copy transfer???
                                    }));
  }
}

/**
 * @brief Spawn subscribers based on the read rosparams for this type
*/
template<typename T> inline
void BridgeType<T>::spawnSubscribers()
{
  for (size_t i=0; i<sub_topics_.size(); i++)
  {
    {
      RCLCPP_INFO(this->get_logger(),
                         "Spawning subscriber  [%s, %s, %d]",
                            sub_topics_[i].c_str(), sub_datapaths_[i].c_str(), sub_freqs_[i]);
      subs_.
          push_back(this->create_subscription<T>(
                     sub_topics_[i], 10,
                     [this, i](const typename T::SharedPtr msg) ->
                     void {
                       if(!comm_.sendToPLC(sub_datapaths_[i], *msg.get()))
                         RCLCPP_ERROR_STREAM(this->get_logger(),
                               sub_topics_[i] << " Failed to send data to PLC at " << sub_datapaths_[i]);
                       }));
    }
  }
}

#endif // BRIDGE_TYPE_H
