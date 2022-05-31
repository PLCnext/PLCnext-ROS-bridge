#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

#include "phoenix_bridge/include_types.h"

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

  /*[[[cog
  import cog
  import sys
  import os

  sys.path.append(os.getcwd()) # Necessary when colcon build invokes this script

  from pydoc import locate
  from phoenix_bridge.param_parser import ParamParser, getResolvedTypeName
  from phoenix_bridge.msg_parser import decomposeRosMsgType, extract_import_names

  params = ParamParser()
  for node in params.nodes_:
    decomposed_fields = decomposeRosMsgType(locate(extract_import_names(node.header_name)))
    cog.outl("template <> inline")
    cog.outl("void castToGrpcObject<{}>({} ros_msg,{} &grpc_object)"
              .format(getResolvedTypeName(node.header_name),
                      getResolvedTypeName(node.header_name),
                      getResolvedTypeName(node.header_name)))
    cog.outl("{")
    for field in decomposed_fields:
      cog.outl("grpc_object.{}= ros_msg.{}; //type:{}".format(field[0], field[0], field[1]))
    cog.outl("}")
    cog.outl(" ")
  ]]]*/
  template <> inline
  void castToGrpcObject<nav_msgs::msg::Odometry>(nav_msgs::msg::Odometry ros_msg,nav_msgs::msg::Odometry &grpc_object)
  {
  grpc_object.header.stamp.sec= ros_msg.header.stamp.sec; //type:int32
  grpc_object.header.stamp.nanosec= ros_msg.header.stamp.nanosec; //type:uint32
  grpc_object.header.frame_id= ros_msg.header.frame_id; //type:string
  grpc_object.child_frame_id= ros_msg.child_frame_id; //type:string
  grpc_object.pose.pose.position.x= ros_msg.pose.pose.position.x; //type:double
  grpc_object.pose.pose.position.y= ros_msg.pose.pose.position.y; //type:double
  grpc_object.pose.pose.position.z= ros_msg.pose.pose.position.z; //type:double
  grpc_object.pose.pose.orientation.x= ros_msg.pose.pose.orientation.x; //type:double
  grpc_object.pose.pose.orientation.y= ros_msg.pose.pose.orientation.y; //type:double
  grpc_object.pose.pose.orientation.z= ros_msg.pose.pose.orientation.z; //type:double
  grpc_object.pose.pose.orientation.w= ros_msg.pose.pose.orientation.w; //type:double
  grpc_object.pose.covariance= ros_msg.pose.covariance; //type:double[36]
  grpc_object.twist.twist.linear.x= ros_msg.twist.twist.linear.x; //type:double
  grpc_object.twist.twist.linear.y= ros_msg.twist.twist.linear.y; //type:double
  grpc_object.twist.twist.linear.z= ros_msg.twist.twist.linear.z; //type:double
  grpc_object.twist.twist.angular.x= ros_msg.twist.twist.angular.x; //type:double
  grpc_object.twist.twist.angular.y= ros_msg.twist.twist.angular.y; //type:double
  grpc_object.twist.twist.angular.z= ros_msg.twist.twist.angular.z; //type:double
  grpc_object.twist.covariance= ros_msg.twist.covariance; //type:double[36]
  }
   
  template <> inline
  void castToGrpcObject<geometry_msgs::msg::Twist>(geometry_msgs::msg::Twist ros_msg,geometry_msgs::msg::Twist &grpc_object)
  {
  grpc_object.linear.x= ros_msg.linear.x; //type:double
  grpc_object.linear.y= ros_msg.linear.y; //type:double
  grpc_object.linear.z= ros_msg.linear.z; //type:double
  grpc_object.angular.x= ros_msg.angular.x; //type:double
  grpc_object.angular.y= ros_msg.angular.y; //type:double
  grpc_object.angular.z= ros_msg.angular.z; //type:double
  }
   
  template <> inline
  void castToGrpcObject<std_msgs::msg::String>(std_msgs::msg::String ros_msg,std_msgs::msg::String &grpc_object)
  {
  grpc_object.data= ros_msg.data; //type:string
  }
   
  //[[[end]]]
}

#endif // CONVERSIONS_HPP
