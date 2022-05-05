#ifndef DUMMY_PHOENIX_COMM_H
#define DUMMY_PHOENIX_COMM_H

#include <string>
/// Include whatever is required for the communication layer. For gRPC this would be the headers generated from proto

/**
 * @brief This will form the communication layer for the birdge b/w ROS and PLC.
 *        Could be gRPC/Shared memory/Unix domain sockets
 *        Each bridge type will instantiate one instance of this communication type
 *          (i.e. one gRPC channel per type, or one shared memory file per type etc..)
 * @todo  Should we instead create one instance (gRPC channel/shared mem file/socket) per port (i.e. sub/pub)?
 *        This could be too much traffic?
 */
template <typename T>
class DummyPhoenixComm
{
public:
  DummyPhoenixComm() {}
  bool sendToPLC(const std::string instance_path, const T& data);
  bool getFromPLC(const std::string instance_path, T& data);
  void init(const std::string address);

private:
  /// Communication layer specific objects. For gRPC this would be grpc::grpc::Channel, grpc::grpc::Context etc..

};

/**
 * @brief Send data to the PLC. Could be to gRPC server.
 * @param instance_path Send data to this path
 * @param data the data to send.
 *        The type is one of the ROS msgs as defined under phoenix_contact/include_types.h
 * @return if sending was succesfull
 */
template<typename T> inline
bool DummyPhoenixComm<T>::sendToPLC(const std::string instance_path, const T &data)
{
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
bool DummyPhoenixComm<T>::getFromPLC(const std::string instance_path, T &data)
{
  return true;
}

/**
 * @brief Initialise the communication layer. Could be creating the gRPC channel from the address
 * @param address Address for the communication layer. Could be IP for gRPC server.
 */
template<typename T> inline
void DummyPhoenixComm<T>::init(const std::string address)
{
}
#endif // DUMMY_PHOENIX_COMM_H
