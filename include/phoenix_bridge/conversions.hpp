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
  /// @todo: Change the grpc_object type to actual grpc::Object type when the library is received
  template <typename T> inline
  void castToGrpcObject(T ros_msg, T &grpc_object)
  {
    (void) ros_msg;
    (void) grpc_object;
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("conversions"),
                "Conversion from type " << typeid(ros_msg).name() << " not implemented!!");
  }

  // Template for the actual grpc type
  template <typename T> inline
  void castToGrpcObject(T ros_msg, ::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object)
  {
    (void) ros_msg;
    (void) grpc_object;
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("conversions"),
                "Conversion from type " << typeid(ros_msg).name() << " not implemented!!");
  }

  // double & bool are used by phoenix IO services
  template <> inline
  void castToGrpcObject<double>(double ros_msg, double &grpc_object)
  {
    grpc_object = ros_msg;
  }

  template <> inline
  void castToGrpcObject<bool>(bool ros_msg, bool &grpc_object)
  {
    grpc_object = ros_msg;
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
    fields.insert(0,(write_item_name, 0, "STRUCT")) # Insert msg name as the uppermost base struct
    cog.outl("template <> inline")
    cog.outl("void castToGrpcObject<{}>({} ros_msg,::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object)"
              .format(getResolvedTypeName(node.header_name),
                      getResolvedTypeName(node.header_name)))

    cog.outl("{")
    cog.outl(" ")
    cog.outl("(void) grpc_object;")
    cog.outl(" ")
    cog.outl("IDataAccessServiceWriteRequest request;")
    cog.outl(" ")
    cog.outl("::Arp::Plc::Gds::Services::Grpc::WriteItem* {}= request.add_data();".format(write_item_name))
    cog.outl("{}->set_portname(\"Arp.Plc.Eclr/MainInstance.ROS_2_PLC_Twist\");".format(write_item_name))
    cog.outl("{}->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);".format(write_item_name))
    cog.outl("")
    for ind in range(1, len(fields)): # skip the 0th element, which is the base struct
        nam = fields[ind][0]
        lvl = fields[ind][1]
        typ = fields[ind][2]

        var_name = fields[ind][0].replace(".","_")
        grpc_typ = get_grpc_type(fields[ind][2])
        upper = get_upper_struct(fields[:ind], lvl-1) # slice till current index, look for first higher struct

        # Line 1 of boilerplate code
        if upper == write_item_name:
            cog.outl("::Arp::Type::Grpc::ObjectType* {} = {}->mutable_value()->mutable_structvalue()->add_structelements();"
                .format(var_name, upper))
        else:
            cog.outl("::Arp::Type::Grpc::ObjectType* {} = {}->mutable_structvalue()->add_structelements();"
                .format(var_name, upper))
        
        # Line 2 of boilerplate code
        if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
            cog.outl("//SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW")
        else:
            cog.outl("{}->set_typecode(::Arp::Type::Grpc::CoreType::{});".format(var_name, grpc_typ))
      
        # Line 3 of boilerplate code
        if typ != "STRUCT":
          if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
              cog.outl("//SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW")
          else:
              cog.outl("{}->set_{}value(ros_msg.{});".format(var_name, typ, nam))
        cog.outl("")

    cog.outl("}")
    cog.outl(" ")
  ]]]*/
  template <> inline
  void castToGrpcObject<nav_msgs::msg::Odometry>(nav_msgs::msg::Odometry ros_msg,::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object)
  {
   
  (void) grpc_object;
   
  IDataAccessServiceWriteRequest request;
   
  ::Arp::Plc::Gds::Services::Grpc::WriteItem* nav_msgs_msg_odometry= request.add_data();
  nav_msgs_msg_odometry->set_portname("Arp.Plc.Eclr/MainInstance.ROS_2_PLC_Twist");
  nav_msgs_msg_odometry->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* header_1 = nav_msgs_msg_odometry->mutable_value()->mutable_structvalue()->add_structelements();
  header_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* stamp_2 = header_1->mutable_structvalue()->add_structelements();
  stamp_2->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* header_stamp_sec = stamp_2->mutable_structvalue()->add_structelements();
  header_stamp_sec->set_typecode(::Arp::Type::Grpc::CoreType::CT_Int32);
  header_stamp_sec->set_int32value(ros_msg.header.stamp.sec);

  ::Arp::Type::Grpc::ObjectType* header_stamp_nanosec = stamp_2->mutable_structvalue()->add_structelements();
  header_stamp_nanosec->set_typecode(::Arp::Type::Grpc::CoreType::CT_Uint32);
  header_stamp_nanosec->set_uint32value(ros_msg.header.stamp.nanosec);

  ::Arp::Type::Grpc::ObjectType* header_frame_id = header_1->mutable_structvalue()->add_structelements();
  header_frame_id->set_typecode(::Arp::Type::Grpc::CoreType::CT_String);
  header_frame_id->set_stringvalue(ros_msg.header.frame_id);

  ::Arp::Type::Grpc::ObjectType* child_frame_id = nav_msgs_msg_odometry->mutable_value()->mutable_structvalue()->add_structelements();
  child_frame_id->set_typecode(::Arp::Type::Grpc::CoreType::CT_String);
  child_frame_id->set_stringvalue(ros_msg.child_frame_id);

  ::Arp::Type::Grpc::ObjectType* pose_1 = nav_msgs_msg_odometry->mutable_value()->mutable_structvalue()->add_structelements();
  pose_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* pose_2 = pose_1->mutable_structvalue()->add_structelements();
  pose_2->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* position_3 = pose_2->mutable_structvalue()->add_structelements();
  position_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* pose_pose_position_x = position_3->mutable_structvalue()->add_structelements();
  pose_pose_position_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_position_x->set_doublevalue(ros_msg.pose.pose.position.x);

  ::Arp::Type::Grpc::ObjectType* pose_pose_position_y = position_3->mutable_structvalue()->add_structelements();
  pose_pose_position_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_position_y->set_doublevalue(ros_msg.pose.pose.position.y);

  ::Arp::Type::Grpc::ObjectType* pose_pose_position_z = position_3->mutable_structvalue()->add_structelements();
  pose_pose_position_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_position_z->set_doublevalue(ros_msg.pose.pose.position.z);

  ::Arp::Type::Grpc::ObjectType* orientation_3 = pose_2->mutable_structvalue()->add_structelements();
  orientation_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_x = orientation_3->mutable_structvalue()->add_structelements();
  pose_pose_orientation_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_orientation_x->set_doublevalue(ros_msg.pose.pose.orientation.x);

  ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_y = orientation_3->mutable_structvalue()->add_structelements();
  pose_pose_orientation_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_orientation_y->set_doublevalue(ros_msg.pose.pose.orientation.y);

  ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_z = orientation_3->mutable_structvalue()->add_structelements();
  pose_pose_orientation_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_orientation_z->set_doublevalue(ros_msg.pose.pose.orientation.z);

  ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_w = orientation_3->mutable_structvalue()->add_structelements();
  pose_pose_orientation_w->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  pose_pose_orientation_w->set_doublevalue(ros_msg.pose.pose.orientation.w);

  ::Arp::Type::Grpc::ObjectType* pose_covariance = pose_1->mutable_structvalue()->add_structelements();
  //SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW
  //SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW

  ::Arp::Type::Grpc::ObjectType* twist_1 = nav_msgs_msg_odometry->mutable_value()->mutable_structvalue()->add_structelements();
  twist_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* twist_2 = twist_1->mutable_structvalue()->add_structelements();
  twist_2->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* linear_3 = twist_2->mutable_structvalue()->add_structelements();
  linear_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* twist_twist_linear_x = linear_3->mutable_structvalue()->add_structelements();
  twist_twist_linear_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_linear_x->set_doublevalue(ros_msg.twist.twist.linear.x);

  ::Arp::Type::Grpc::ObjectType* twist_twist_linear_y = linear_3->mutable_structvalue()->add_structelements();
  twist_twist_linear_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_linear_y->set_doublevalue(ros_msg.twist.twist.linear.y);

  ::Arp::Type::Grpc::ObjectType* twist_twist_linear_z = linear_3->mutable_structvalue()->add_structelements();
  twist_twist_linear_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_linear_z->set_doublevalue(ros_msg.twist.twist.linear.z);

  ::Arp::Type::Grpc::ObjectType* angular_3 = twist_2->mutable_structvalue()->add_structelements();
  angular_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* twist_twist_angular_x = angular_3->mutable_structvalue()->add_structelements();
  twist_twist_angular_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_angular_x->set_doublevalue(ros_msg.twist.twist.angular.x);

  ::Arp::Type::Grpc::ObjectType* twist_twist_angular_y = angular_3->mutable_structvalue()->add_structelements();
  twist_twist_angular_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_angular_y->set_doublevalue(ros_msg.twist.twist.angular.y);

  ::Arp::Type::Grpc::ObjectType* twist_twist_angular_z = angular_3->mutable_structvalue()->add_structelements();
  twist_twist_angular_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  twist_twist_angular_z->set_doublevalue(ros_msg.twist.twist.angular.z);

  ::Arp::Type::Grpc::ObjectType* twist_covariance = twist_1->mutable_structvalue()->add_structelements();
  //SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW
  //SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW

  }
   
  template <> inline
  void castToGrpcObject<geometry_msgs::msg::Twist>(geometry_msgs::msg::Twist ros_msg,::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object)
  {
   
  (void) grpc_object;
   
  IDataAccessServiceWriteRequest request;
   
  ::Arp::Plc::Gds::Services::Grpc::WriteItem* geometry_msgs_msg_twist= request.add_data();
  geometry_msgs_msg_twist->set_portname("Arp.Plc.Eclr/MainInstance.ROS_2_PLC_Twist");
  geometry_msgs_msg_twist->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* linear_1 = geometry_msgs_msg_twist->mutable_value()->mutable_structvalue()->add_structelements();
  linear_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* linear_x = linear_1->mutable_structvalue()->add_structelements();
  linear_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  linear_x->set_doublevalue(ros_msg.linear.x);

  ::Arp::Type::Grpc::ObjectType* linear_y = linear_1->mutable_structvalue()->add_structelements();
  linear_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  linear_y->set_doublevalue(ros_msg.linear.y);

  ::Arp::Type::Grpc::ObjectType* linear_z = linear_1->mutable_structvalue()->add_structelements();
  linear_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  linear_z->set_doublevalue(ros_msg.linear.z);

  ::Arp::Type::Grpc::ObjectType* angular_1 = geometry_msgs_msg_twist->mutable_value()->mutable_structvalue()->add_structelements();
  angular_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* angular_x = angular_1->mutable_structvalue()->add_structelements();
  angular_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  angular_x->set_doublevalue(ros_msg.angular.x);

  ::Arp::Type::Grpc::ObjectType* angular_y = angular_1->mutable_structvalue()->add_structelements();
  angular_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  angular_y->set_doublevalue(ros_msg.angular.y);

  ::Arp::Type::Grpc::ObjectType* angular_z = angular_1->mutable_structvalue()->add_structelements();
  angular_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
  angular_z->set_doublevalue(ros_msg.angular.z);

  }
   
  template <> inline
  void castToGrpcObject<std_msgs::msg::String>(std_msgs::msg::String ros_msg,::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object)
  {
   
  (void) grpc_object;
   
  IDataAccessServiceWriteRequest request;
   
  ::Arp::Plc::Gds::Services::Grpc::WriteItem* std_msgs_msg_string= request.add_data();
  std_msgs_msg_string->set_portname("Arp.Plc.Eclr/MainInstance.ROS_2_PLC_Twist");
  std_msgs_msg_string->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  ::Arp::Type::Grpc::ObjectType* data = std_msgs_msg_string->mutable_value()->mutable_structvalue()->add_structelements();
  data->set_typecode(::Arp::Type::Grpc::CoreType::CT_String);
  data->set_stringvalue(ros_msg.data);

  }
   
  //[[[end]]]
}

#endif // CONVERSIONS_HPP
