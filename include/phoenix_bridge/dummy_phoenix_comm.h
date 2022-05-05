#ifndef DUMMY_PHOENIX_COMM_H
#define DUMMY_PHOENIX_COMM_H

#include <string>

template <typename T>
class DummyPhoenixComm
{
public:
  DummyPhoenixComm() {}
  bool sendToPLC(const std::string instance_path, const T& data);
  bool getFromPLC(const std::string instance_path, T& data);

};
#endif // DUMMY_PHOENIX_COMM_H
