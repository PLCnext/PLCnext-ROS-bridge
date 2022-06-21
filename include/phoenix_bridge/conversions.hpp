#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

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


namespace conversions
{
  /// Base template function. Implement template specializations for each type which has to be handled.
  /// If type specilization not implemented but invoked, prints this error msg (@todo: and also kill node)
  template <typename T> inline
  void packWriteItem(::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object, T data_to_pack)
  {
    (void) data_to_pack;
    (void) grpc_object;
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("conversions"),
                "Conversion from type " << typeid(data_to_pack).name() << " not implemented!!");
  }

  // double & bool are used by phoenix IO services
  template <> inline
  void packWriteItem<double>(::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object, double data_to_pack)
  {
    ::Arp::Type::Grpc::ObjectType* data = grpc_object->mutable_value()->mutable_structvalue()->add_structelements();
    data->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    data->set_doublevalue(data_to_pack);
  }

  template <> inline
  void packWriteItem<bool>(::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object, bool data_to_pack)
  {
    ::Arp::Type::Grpc::ObjectType* data = grpc_object->mutable_value()->mutable_structvalue()->add_structelements();
    data->set_typecode(::Arp::Type::Grpc::CoreType::CT_Boolean);
    data->set_boolvalue(data_to_pack);
  }

  /*[[[cog
  import cog
  import sys
  import os

  sys.path.append(os.getcwd()) # Necessary when colcon build invokes this script

  from pydoc import locate
  from phoenix_bridge.param_parser import ParamParser, getResolvedTypeName
  from phoenix_bridge.msg_parser import decompose_ros_msg_type, extract_import_names, get_grpc_type, get_upper_struct

  params = ParamParser()
  for node in params.nodes_:
    fields = decompose_ros_msg_type(locate(extract_import_names(node.header_name)))
    write_item_name = node.header_name.replace("/","_")
    fields.insert(0,("grpc_object", 0, "STRUCT")) # Insert the received grpc_object as the uppermost base struct
    cog.outl("template <> inline")
    cog.outl("void packWriteItem<{}>(::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object, {} data_to_pack)"
              .format(getResolvedTypeName(node.header_name),
                      getResolvedTypeName(node.header_name)))

    cog.outl("{")
    for ind in range(1, len(fields)): # skip the 0th element, which is the base struct
        nam = fields[ind][0]
        lvl = fields[ind][1]
        typ = fields[ind][2]

        var_name = fields[ind][0].replace(".","_")
        grpc_typ = get_grpc_type(fields[ind][2])
        upper = get_upper_struct(fields[:ind], lvl-1) # slice till current index, look for first higher struct

        # Line 1 of boilerplate code
        if upper == "grpc_object":
            cog.outl("::Arp::Type::Grpc::ObjectType* {} = {}->mutable_value()->mutable_structvalue()->add_structelements();"
                .format(var_name, upper))
        else:
            cog.outl("::Arp::Type::Grpc::ObjectType* {} = {}->mutable_structvalue()->add_structelements();"
                .format(var_name, upper))
        
        # Line 2 of boilerplate code
        if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
            cog.outl("(void) {};".format(var_name))
            cog.outl("//SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW")
        else:
            cog.outl("{}->set_typecode(::Arp::Type::Grpc::CoreType::{});".format(var_name, grpc_typ))
      
        # Line 3 of boilerplate code
        if typ != "STRUCT":
          if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
              cog.outl("//SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW")
          else:
              cog.outl("{}->set_{}value(data_to_pack.{});".format(var_name, typ, nam))
        cog.outl("")

    cog.outl("}")
    cog.outl(" ")
  ]]]*/
  template <> inline
  void packWriteItem<nav_msgs::msg::Odometry>(::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object, nav_msgs::msg::Odometry data_to_pack)
  {
  ::Arp::Type::Grpc::ObjectType* header_1 = grpc_object->mutable_value()->mutable_structvalue()->add_structelements();
  header_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* stamp_2 = header_1->mutable_structvalue()->add_structelements();
  stamp_2->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* header_stamp_sec = stamp_2->mutable_structvalue()->add_structelements();
  header_stamp_sec->set_typecode(::Arp::Type::Grpc::CoreType::CT_Int32);
  header_stamp_sec->set_int32value(data_to_pack.header.stamp.sec);

  ::Arp::Type::Grpc::ObjectType* header_stamp_nanosec = stamp_2->mutable_structvalue()->add_structelements();
  header_stamp_nanosec->set_typecode(::Arp::Type::Grpc::CoreType::CT_Uint32);
  header_stamp_nanosec->set_uint32value(data_to_pack.header.stamp.nanosec);

  ::Arp::Type::Grpc::ObjectType* header_frame_id = header_1->mutable_structvalue()->add_structelements();
  header_frame_id->set_typecode(::Arp::Type::Grpc::CoreType::CT_String);
  header_frame_id->set_stringvalue(data_to_pack.header.frame_id);

  ::Arp::Type::Grpc::ObjectType* child_frame_id = grpc_object->mutable_value()->mutable_structvalue()->add_structelements();
  child_frame_id->set_typecode(::Arp::Type::Grpc::CoreType::CT_String);
  child_frame_id->set_stringvalue(data_to_pack.child_frame_id);

  ::Arp::Type::Grpc::ObjectType* pose_1 = grpc_object->mutable_value()->mutable_structvalue()->add_structelements();
  pose_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* pose_2 = pose_1->mutable_structvalue()->add_structelements();
  pose_2->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* position_3 = pose_2->mutable_structvalue()->add_structelements();
  position_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* pose_pose_position_x = position_3->mutable_structvalue()->add_structelements();
  pose_pose_position_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_position_x->set_doublevalue(data_to_pack.pose.pose.position.x);

  ::Arp::Type::Grpc::ObjectType* pose_pose_position_y = position_3->mutable_structvalue()->add_structelements();
  pose_pose_position_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_position_y->set_doublevalue(data_to_pack.pose.pose.position.y);

  ::Arp::Type::Grpc::ObjectType* pose_pose_position_z = position_3->mutable_structvalue()->add_structelements();
  pose_pose_position_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_position_z->set_doublevalue(data_to_pack.pose.pose.position.z);

  ::Arp::Type::Grpc::ObjectType* orientation_3 = pose_2->mutable_structvalue()->add_structelements();
  orientation_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_x = orientation_3->mutable_structvalue()->add_structelements();
  pose_pose_orientation_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_orientation_x->set_doublevalue(data_to_pack.pose.pose.orientation.x);

  ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_y = orientation_3->mutable_structvalue()->add_structelements();
  pose_pose_orientation_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_orientation_y->set_doublevalue(data_to_pack.pose.pose.orientation.y);

  ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_z = orientation_3->mutable_structvalue()->add_structelements();
  pose_pose_orientation_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_orientation_z->set_doublevalue(data_to_pack.pose.pose.orientation.z);

  ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_w = orientation_3->mutable_structvalue()->add_structelements();
  pose_pose_orientation_w->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_orientation_w->set_doublevalue(data_to_pack.pose.pose.orientation.w);

  ::Arp::Type::Grpc::ObjectType* pose_covariance = pose_1->mutable_structvalue()->add_structelements();
  (void) pose_covariance;
  //SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW
  //SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW

  ::Arp::Type::Grpc::ObjectType* twist_1 = grpc_object->mutable_value()->mutable_structvalue()->add_structelements();
  twist_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* twist_2 = twist_1->mutable_structvalue()->add_structelements();
  twist_2->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* linear_3 = twist_2->mutable_structvalue()->add_structelements();
  linear_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* twist_twist_linear_x = linear_3->mutable_structvalue()->add_structelements();
  twist_twist_linear_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_linear_x->set_doublevalue(data_to_pack.twist.twist.linear.x);

  ::Arp::Type::Grpc::ObjectType* twist_twist_linear_y = linear_3->mutable_structvalue()->add_structelements();
  twist_twist_linear_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_linear_y->set_doublevalue(data_to_pack.twist.twist.linear.y);

  ::Arp::Type::Grpc::ObjectType* twist_twist_linear_z = linear_3->mutable_structvalue()->add_structelements();
  twist_twist_linear_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_linear_z->set_doublevalue(data_to_pack.twist.twist.linear.z);

  ::Arp::Type::Grpc::ObjectType* angular_3 = twist_2->mutable_structvalue()->add_structelements();
  angular_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* twist_twist_angular_x = angular_3->mutable_structvalue()->add_structelements();
  twist_twist_angular_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_angular_x->set_doublevalue(data_to_pack.twist.twist.angular.x);

  ::Arp::Type::Grpc::ObjectType* twist_twist_angular_y = angular_3->mutable_structvalue()->add_structelements();
  twist_twist_angular_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_angular_y->set_doublevalue(data_to_pack.twist.twist.angular.y);

  ::Arp::Type::Grpc::ObjectType* twist_twist_angular_z = angular_3->mutable_structvalue()->add_structelements();
  twist_twist_angular_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_angular_z->set_doublevalue(data_to_pack.twist.twist.angular.z);

  ::Arp::Type::Grpc::ObjectType* twist_covariance = twist_1->mutable_structvalue()->add_structelements();
  (void) twist_covariance;
  //SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW
  //SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW

  }
   
  template <> inline
  void packWriteItem<geometry_msgs::msg::Twist>(::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object, geometry_msgs::msg::Twist data_to_pack)
  {
  ::Arp::Type::Grpc::ObjectType* linear_1 = grpc_object->mutable_value()->mutable_structvalue()->add_structelements();
  linear_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* linear_x = linear_1->mutable_structvalue()->add_structelements();
  linear_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  linear_x->set_doublevalue(data_to_pack.linear.x);

  ::Arp::Type::Grpc::ObjectType* linear_y = linear_1->mutable_structvalue()->add_structelements();
  linear_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  linear_y->set_doublevalue(data_to_pack.linear.y);

  ::Arp::Type::Grpc::ObjectType* linear_z = linear_1->mutable_structvalue()->add_structelements();
  linear_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  linear_z->set_doublevalue(data_to_pack.linear.z);

  ::Arp::Type::Grpc::ObjectType* angular_1 = grpc_object->mutable_value()->mutable_structvalue()->add_structelements();
  angular_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* angular_x = angular_1->mutable_structvalue()->add_structelements();
  angular_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  angular_x->set_doublevalue(data_to_pack.angular.x);

  ::Arp::Type::Grpc::ObjectType* angular_y = angular_1->mutable_structvalue()->add_structelements();
  angular_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  angular_y->set_doublevalue(data_to_pack.angular.y);

  ::Arp::Type::Grpc::ObjectType* angular_z = angular_1->mutable_structvalue()->add_structelements();
  angular_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  angular_z->set_doublevalue(data_to_pack.angular.z);

  }
   
  template <> inline
  void packWriteItem<std_msgs::msg::String>(::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object, std_msgs::msg::String data_to_pack)
  {
  ::Arp::Type::Grpc::ObjectType* data = grpc_object->mutable_value()->mutable_structvalue()->add_structelements();
  data->set_typecode(::Arp::Type::Grpc::CoreType::CT_String);
  data->set_stringvalue(data_to_pack.data);

  }
   
  //[[[end]]]
}

#endif // CONVERSIONS_HPP
