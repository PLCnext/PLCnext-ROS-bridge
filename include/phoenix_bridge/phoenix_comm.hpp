#ifndef PHOENIX_BRIDGE__PHOENIX_COMM_HPP_
#define PHOENIX_BRIDGE__PHOENIX_COMM_HPP_

#include <iostream>
#include <memory>
#include <string>

#include "phoenix_bridge/read_conversions.hpp"
#include "phoenix_bridge/write_conversions.hpp"

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
  bool sendToPLC(const std::string instance_path, const T & data);
  bool getFromPLC(const std::string instance_path, T & data);
  void init(const std::string address);

private:
  std::unique_ptr<IDataAccessService::Stub> stub_;  /// Stub to access underlying grpc functionality
};

/**
 * @brief Constructor
 *
 * @tparam T Template type
 */
template <typename T>
inline PhoenixComm<T>::PhoenixComm()
{
}

/**
 * @brief Send data to the PLC through grpc.
 * @param instance_path Send data to this path in the PLC GDS
 * @param data the data to send. The type is one of the ROS msgs as defined under phoenix_contact/include_types.h, or a base type.
 * @return if sending was succesfull
 * @todo Room for improvement? Error catching? Performance optimisation?
 */
template <typename T>
inline bool PhoenixComm<T>::sendToPLC(const std::string instance_path, const T & data)
{
  IDataAccessServiceWriteRequest request;

  ::Arp::Plc::Gds::Services::Grpc::WriteItem * grpc_object = request.add_data();
  grpc_object->set_portname(instance_path);
  grpc_object->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  conversions::packWriteItem(grpc_object, data);

  ClientContext context;
  IDataAccessServiceWriteResponse reply;
  Status status = stub_->Write(&context, request, &reply);

  if (status.ok()) {
    return true;
  } else {
    std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    return false;
  }
  return true;
}

/**
 * @brief Get data from the PLC through the gRPC server
 * @param instance_path Get data from the PLC GDS at this address
 * @param data Cast and write the received data into this variable.
 *        The type is one of the ROS msgs as defined under phoenix_contact/include_types.h or a basic type
 * @return if retrieval was succesfull
 */
template <typename T>
inline bool PhoenixComm<T>::getFromPLC(const std::string instance_path, T & data)
{
  IDataAccessServiceReadRequest request;
  ClientContext context;
  IDataAccessServiceReadResponse reply;

  request.add_portnames(instance_path);
  Status status = stub_->Read(&context, request, &reply);

  if (status.ok()) {
    ObjectType grpc_object = reply._returnvalue(0).value();
    conversions::unpackReadObject(grpc_object, data);
    return true;
  } else {
    std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    return false;
  }
}

/**
 * @brief Initialise the communication layer by creating the gRPC channel from the address
 * @param address Unix socket address of channel
 */
template <typename T>
inline void PhoenixComm<T>::init(const std::string address)
{
  stub_ =
    IDataAccessService::NewStub(grpc::CreateChannel(address, grpc::InsecureChannelCredentials()));
}
#endif  // PHOENIX_BRIDGE__PHOENIX_COMM_HPP_
