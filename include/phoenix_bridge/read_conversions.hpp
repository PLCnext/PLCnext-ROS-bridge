#ifndef READ_CONVERSIONS_HPP
#define READ_CONVERSIONS_HPP

#include "phoenix_bridge/include_types.h"

#include <grpcpp/grpcpp.h>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/dynamic_message.h>

#include "phoenix_bridge/ServiceStubs/Plc/Gds/IDataAccessService.grpc.pb.h"
#include "phoenix_bridge/ServiceStubs/ArpTypes.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::Status;

using Arp::Type::Grpc::ObjectType;

using Arp::Plc::Gds::Services::Grpc::IDataAccessService;
using Arp::Plc::Gds::Services::Grpc::IDataAccessServiceReadRequest;
using Arp::Plc::Gds::Services::Grpc::IDataAccessServiceReadResponse;
using Arp::Plc::Gds::Services::Grpc::IDataAccessServiceReadSingleRequest;
using Arp::Plc::Gds::Services::Grpc::IDataAccessServiceReadSingleResponse;
using Arp::Plc::Gds::Services::Grpc::IDataAccessServiceWriteRequest;
using Arp::Plc::Gds::Services::Grpc::IDataAccessServiceWriteResponse;
using Arp::Plc::Gds::Services::Grpc::IDataAccessServiceWriteSingleRequest;
using Arp::Plc::Gds::Services::Grpc::IDataAccessServiceWriteSingleResponse;

/**
 * @brief Provides templated functions to unpackpack data from grpc::ObjectType, which should be extracted from a
 * gRPC PLC read service call, into ros msg (or any basic type)
 * The templated function specializations are generated by cog at build time for each of the types specified in the
 * parameters file.
 */
namespace conversions
{
  /**
   * @brief Base template function to unpack data from a ReadResponse object
   * Implement template specializations for each type which has to be handled.
   * If type specilization not implemented but invoked, prints this error msg (@todo: and also kill node)
   * @param grpc_object grpc::ObjectType, extracted from the read response object, from which data needs to be unpacked
   * @param unpack_to_data The ros msg data variable into which the read resposne has to be unpacked. Can also be base types.
   */
  template <typename T> inline
  void unpackReadObject(const ObjectType &grpc_object, T& unpack_to_data)
  {
    (void) unpack_to_data;
    (void) grpc_object;
    ROS_ERROR_STREAM_ONCE("Conversion to type " << typeid(unpack_to_data).name() << " not implemented!!");
  }

  /**
   * @brief Bool specialization
   */
  template <>
  inline void unpackReadObject<bool>(const ObjectType &grpc_object, bool& unpack_to_data)
  {
    unpack_to_data = grpc_object.boolvalue();
  }

  /**
   * @brief Double specialization
   */
  template <>
  inline void unpackReadObject<double>(const ObjectType &grpc_object, double& unpack_to_data)
  {
    unpack_to_data = grpc_object.doublevalue();
  }

  ///
  /// The following section is generated by cog at build time. Manual edits will be lost when rebuilt.
  ///

  /*[[[cog
  import cog
  import sys
  import os

  sys.path.append(os.getcwd()) # Necessary when colcon build invokes this script

  from pydoc import locate
  from src.parsers.msg_parser import decompose_ros_msg_type, get_grpc_type, get_upper_struct
  from src.parsers.param_parser import ParamParser, header_format, namespace_format

  params = ParamParser()
  for a_type in params.types_:
    cog.outl("//----------"+header_format(a_type)+"---------------")
    fields = decompose_ros_msg_type(locate(a_type[0]+".msg."+a_type[1]))
    fields.insert(0,("grpc_object", 0, "STRUCT")) # Insert the received grpc_object as the uppermost base struct
    parent_dict = {}

    cog.outl("template <>")
    cog.outl("inline void unpackReadObject<{}>(const ObjectType &grpc_object, {}& unpack_to_data)"
              .format(namespace_format(a_type),
                      namespace_format(a_type)))

    cog.outl("{")
    for ind in range(1, len(fields)): # skip the 0th element, which is the base struct
        nam = fields[ind][0]
        lvl = fields[ind][1]
        typ = fields[ind][2]

        # Handling special types
        # @TODO: How to handle time properly? CT_XX? set_XXvalue()??
        typ = "double" if typ=="float64" else typ

        var_name = fields[ind][0].replace(".","_")
        grpc_typ = get_grpc_type(fields[ind][2])
        upper = get_upper_struct(fields[:ind], lvl-1) # slice till current index, look for first higher struct

        child_index = 0
        if not upper in parent_dict.keys():
            parent_dict[upper] = 0
        else:
            child_index = parent_dict[upper]
        parent_dict[upper] += 1

        cog.outl("  ObjectType {} = {}.structvalue().structelements({});".format(var_name, upper, child_index))
        if typ != "STRUCT":
            if typ=="time":
                cog.outl("  //SKIPPING TIME TYPE FOR NOW")
                cog.outl("")
                continue
            if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
                array_typ = typ.split('[')[0] # Get the type of the array
                array_typ = "double" if array_typ=="float64" else array_typ
                cog.outl("  for (int i = 0; i < {}.arrayvalue().arrayelements_size(); i++)".format(var_name))
                cog.outl("  {")
                cog.outl("    unpack_to_data.{}[i] = {}.arrayvalue().arrayelements(i).{}value();".format(nam,var_name,array_typ))
                cog.outl("  }")
            else:
                cog.outl("  unpack_to_data.{} = {}.{}value();".format(nam, var_name, typ))
        cog.outl("")

    cog.outl("}")
    cog.outl("")
  ]]]*/
  //----------nav_msgs/Odometry---------------
  template <>
  inline void unpackReadObject<nav_msgs::Odometry>(const ObjectType &grpc_object, nav_msgs::Odometry& unpack_to_data)
  {
    ObjectType header_1 = grpc_object.structvalue().structelements(0);

    ObjectType header_seq = header_1.structvalue().structelements(0);
    unpack_to_data.header.seq = header_seq.uint32value();

    ObjectType header_stamp = header_1.structvalue().structelements(1);
    //SKIPPING TIME TYPE FOR NOW

    ObjectType header_frame_id = header_1.structvalue().structelements(2);
    unpack_to_data.header.frame_id = header_frame_id.stringvalue();

    ObjectType child_frame_id = grpc_object.structvalue().structelements(1);
    unpack_to_data.child_frame_id = child_frame_id.stringvalue();

    ObjectType pose_1 = grpc_object.structvalue().structelements(2);

    ObjectType pose_2 = pose_1.structvalue().structelements(0);

    ObjectType position_3 = pose_2.structvalue().structelements(0);

    ObjectType pose_pose_position_x = position_3.structvalue().structelements(0);
    unpack_to_data.pose.pose.position.x = pose_pose_position_x.doublevalue();

    ObjectType pose_pose_position_y = position_3.structvalue().structelements(1);
    unpack_to_data.pose.pose.position.y = pose_pose_position_y.doublevalue();

    ObjectType pose_pose_position_z = position_3.structvalue().structelements(2);
    unpack_to_data.pose.pose.position.z = pose_pose_position_z.doublevalue();

    ObjectType orientation_3 = pose_2.structvalue().structelements(1);

    ObjectType pose_pose_orientation_x = orientation_3.structvalue().structelements(0);
    unpack_to_data.pose.pose.orientation.x = pose_pose_orientation_x.doublevalue();

    ObjectType pose_pose_orientation_y = orientation_3.structvalue().structelements(1);
    unpack_to_data.pose.pose.orientation.y = pose_pose_orientation_y.doublevalue();

    ObjectType pose_pose_orientation_z = orientation_3.structvalue().structelements(2);
    unpack_to_data.pose.pose.orientation.z = pose_pose_orientation_z.doublevalue();

    ObjectType pose_pose_orientation_w = orientation_3.structvalue().structelements(3);
    unpack_to_data.pose.pose.orientation.w = pose_pose_orientation_w.doublevalue();

    ObjectType pose_covariance = pose_1.structvalue().structelements(1);
    for (int i = 0; i < pose_covariance.arrayvalue().arrayelements_size(); i++)
    {
      unpack_to_data.pose.covariance[i] = pose_covariance.arrayvalue().arrayelements(i).doublevalue();
    }

    ObjectType twist_1 = grpc_object.structvalue().structelements(3);

    ObjectType twist_2 = twist_1.structvalue().structelements(0);

    ObjectType linear_3 = twist_2.structvalue().structelements(0);

    ObjectType twist_twist_linear_x = linear_3.structvalue().structelements(0);
    unpack_to_data.twist.twist.linear.x = twist_twist_linear_x.doublevalue();

    ObjectType twist_twist_linear_y = linear_3.structvalue().structelements(1);
    unpack_to_data.twist.twist.linear.y = twist_twist_linear_y.doublevalue();

    ObjectType twist_twist_linear_z = linear_3.structvalue().structelements(2);
    unpack_to_data.twist.twist.linear.z = twist_twist_linear_z.doublevalue();

    ObjectType angular_3 = twist_2.structvalue().structelements(1);

    ObjectType twist_twist_angular_x = angular_3.structvalue().structelements(0);
    unpack_to_data.twist.twist.angular.x = twist_twist_angular_x.doublevalue();

    ObjectType twist_twist_angular_y = angular_3.structvalue().structelements(1);
    unpack_to_data.twist.twist.angular.y = twist_twist_angular_y.doublevalue();

    ObjectType twist_twist_angular_z = angular_3.structvalue().structelements(2);
    unpack_to_data.twist.twist.angular.z = twist_twist_angular_z.doublevalue();

    ObjectType twist_covariance = twist_1.structvalue().structelements(1);
    for (int i = 0; i < twist_covariance.arrayvalue().arrayelements_size(); i++)
    {
      unpack_to_data.twist.covariance[i] = twist_covariance.arrayvalue().arrayelements(i).doublevalue();
    }

  }

  //----------geometry_msgs/Twist---------------
  template <>
  inline void unpackReadObject<geometry_msgs::Twist>(const ObjectType &grpc_object, geometry_msgs::Twist& unpack_to_data)
  {
    ObjectType linear_1 = grpc_object.structvalue().structelements(0);

    ObjectType linear_x = linear_1.structvalue().structelements(0);
    unpack_to_data.linear.x = linear_x.doublevalue();

    ObjectType linear_y = linear_1.structvalue().structelements(1);
    unpack_to_data.linear.y = linear_y.doublevalue();

    ObjectType linear_z = linear_1.structvalue().structelements(2);
    unpack_to_data.linear.z = linear_z.doublevalue();

    ObjectType angular_1 = grpc_object.structvalue().structelements(1);

    ObjectType angular_x = angular_1.structvalue().structelements(0);
    unpack_to_data.angular.x = angular_x.doublevalue();

    ObjectType angular_y = angular_1.structvalue().structelements(1);
    unpack_to_data.angular.y = angular_y.doublevalue();

    ObjectType angular_z = angular_1.structvalue().structelements(2);
    unpack_to_data.angular.z = angular_z.doublevalue();

  }

  //----------std_msgs/String---------------
  template <>
  inline void unpackReadObject<std_msgs::String>(const ObjectType &grpc_object, std_msgs::String& unpack_to_data)
  {
    ObjectType data = grpc_object.structvalue().structelements(0);
    unpack_to_data.data = data.stringvalue();

  }

  //[[[end]]]

  ///
  /// The above section is generated by cog at build time. Manual edits will be lost when rebuilt.
  ///

}

#endif // READ_CONVERSIONS_HPP