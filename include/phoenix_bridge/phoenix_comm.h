#ifndef PHOENIX_COMM_H
#define PHOENIX_COMM_H

#include <string>
#include <iostream>

#include "phoenix_bridge/conversions.hpp"

/**
 * @brief This will form the communication layer for the birdge b/w ROS and PLC.
 *        Implemented with gRPC
 *        Each bridge type will instantiate one instance of a gRPC channel per type
 */
template <typename T>
class PhoenixComm
{
public:
  PhoenixComm();
  bool sendToPLC(const std::string instance_path, const T& data);
  bool getFromPLC(const std::string instance_path, T& data);
  void init(const std::string address);

private:
  /// Communication layer specific objects. For gRPC this would be grpc::grpc::Channel, grpc::grpc::Context etc..
  std::unique_ptr<IDataAccessService::Stub> stub_;

};

template<typename T> inline
PhoenixComm<T>::PhoenixComm()
{

}

/**
 * @brief Send data to the PLC. Could be to gRPC server.
 * @param instance_path Send data to this path
 * @param data the data to send.
 *        The type is one of the ROS msgs as defined under phoenix_contact/include_types.h
 * @return if sending was succesfull
 */
template<typename T> inline
bool PhoenixComm<T>::sendToPLC(const std::string instance_path, const T &data)
{
  (void) data;
  IDataAccessServiceWriteRequest request;
  
  // Instance path in demo = "Arp.Plc.Eclr/MainInstance.ROS_2_PLC_Twist"
  ::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object = request.add_data();
  grpc_object->set_portname(instance_path);
  grpc_object->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  conversions::packWriteItem(grpc_object, data); 

  ClientContext context;
  IDataAccessServiceWriteResponse reply;
  Status status = stub_->Write(&context, request, &reply);

  if (status.ok()) {
      return true;
  }
  else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return false;
  }
  return true;
}

/**
 * @brief Get data from the PLC. Could be from gRPC server
 * @param instance_path Get data from gRPC server at this address
 * @param data Cast and write the received data into this variable.
 *        The type is one of the ROS msgs as defined under phoenix_contact/include_types.h
 * @return if retrieval was succesfull
 */
template<typename T> inline
bool PhoenixComm<T>::getFromPLC(const std::string instance_path, T &data)
{
  (void) instance_path;
  (void) data;
  
  return true;
}

/**
 * @brief Initialise the communication layer. Could be creating the gRPC channel from the address
 * @param address Address for the communication layer. Could be IP for gRPC server.
 */
template<typename T> inline
void PhoenixComm<T>::init(const std::string address)
{
  // Example address = unix:/run/plcnext/grpc.sock. Read from parameter file during cosntruction.
  stub_ = IDataAccessService::NewStub(grpc::CreateChannel(address, grpc::InsecureChannelCredentials()));
}
#endif // phoenix_comm_H
