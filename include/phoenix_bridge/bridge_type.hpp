#ifndef BRIDGE_TYPE_H
#define BRIDGE_TYPE_H

#include "phoenix_bridge/include_types.h"
#include "phoenix_bridge/params_struct.hpp"
#include "phoenix_bridge/dummy_phoenix_comm.h"

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
  void getPortParams();
  void init();

private:
  std::vector<PortParams> pub_params_; /// Parameters of publishers to create
  std::vector<PortParams> sub_params_; /// Parameters of subscribers to create
  void publisherFunction(typename rclcpp::Publisher<T>::SharedPtr pub, std::string datapath);
  DummyPhoenixComm<T> comm_;
  std::vector<std::string> pub_topics_, pub_datapaths_;
  std::vector<std::string> sub_topics_, sub_datapaths_;
  std::vector<long int> pub_freqs_, sub_freqs_;
  std::vector<rclcpp::TimerBase::SharedPtr> pub_timers_;
  std::vector<typename rclcpp::Publisher<T>::SharedPtr> pubs_;
};

template<typename T> inline
BridgeType<T>::BridgeType(std::string node_name) :
  Node(node_name)
{
  init();
}

/**
 * @brief Read the params to create ports, spawn pubs and subs
 * @param param_name The simple name for the parameters of this type ex: "odometry" or "twist"
 */
template<typename T> inline
void BridgeType<T>::init()
{

  getPortParams();

  /// Initialise communication layer
  comm_.init(this->get_parameter("grpc.address").as_string());

  /// Spawn pubs
  for (size_t i=0; i<pub_topics_.size(); i++)
  {
    RCLCPP_INFO(this->get_logger(),
                       "Spawning publisher  [%s, %s, %d]", pub_topics_[i].c_str(), pub_datapaths_[i].c_str(), pub_freqs_[i]);

    typename rclcpp::Publisher<T>::SharedPtr pub = this->create_publisher<T>(pub_topics_[i], 1000);
    pubs_.push_back(pub);
    std::function<void()> fcn = std::bind(&BridgeType::publisherFunction, this, pub, pub_datapaths_[i]);

    pub_timers_.push_back(this->create_wall_timer(std::chrono::milliseconds(1000/pub_freqs_[i]),fcn));

//    std::thread thr(std::bind(&BridgeType::publisherFunction,
//                              this, pub_topics_[i], pub_datapaths_[i]));
//    thr.detach(); // Thread nust ensure graceful shutdown of itself since it is detached
  }

//  /// Spawn subs
//  for (auto port:sub_params_)
//  {
//    subscribers_.
//        push_back(nh_.subscribe<T>(port.name_, 10,
//                                    [this, port](const boost::shared_ptr<const T> msg) ->
//                                    void {
//                                      if(!comm_.sendToPLC(port.datapath_, *msg.get()))
//                                        ROS_ERROR_STREAM(port.name_ << " Failed to send data to PLC at " << port.datapath_);
//                                    }
//                                   ));
//  }
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

  RCLCPP_INFO(this->get_logger(), "Publishers: ");
  for (size_t i=0; i<pub_topics_.size(); i++)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "  [" << pub_topics_[i] << ", " << pub_datapaths_[i] << ", " << pub_freqs_[i] << "]" );
  }

  RCLCPP_INFO(this->get_logger(), "Subscribers: ");
  for (size_t i=0; i<sub_topics_.size(); i++)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "  [" << sub_topics_[i] << ", " << sub_datapaths_[i] << ", " << sub_freqs_[i] << "]" );
  }
  RCLCPP_INFO(this->get_logger(), "");
}

/**
 * @brief Function bound to a thread per publisher per BridgeType. Get data from PLC and publish at parametrised frequency
 * @param port PortParam object, specifying which datapath to query data for, which topic to publish on and at what frequency
 */
template<typename T> inline
void BridgeType<T>::publisherFunction(typename rclcpp::Publisher<T>::SharedPtr pub, std::string datapath)
{
  T msg_rcvd;
  comm_.getFromPLC(datapath, msg_rcvd);
  typename T::SharedPtr msg_ptr = std::make_shared<T>(msg_rcvd);
  pub->publish(msg_rcvd); // is this 0 copy transfer???
}

#endif // BRIDGE_TYPE_H
