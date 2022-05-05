#ifndef PARAMS_STRUCT_H
#define PARAMS_STRUCT_H

#include <string>

struct PortParams
{
  std::string name_;
  std::string datapath_;
  int frequency_;
  PortParams(std::string name, std::string datapath, int freq):
    name_(name),
    datapath_(datapath),
    frequency_(freq)
    {}
};

#endif // PARAMS_STRUCT_H
