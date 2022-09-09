//
// Copyright 2022 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include <google/protobuf/text_format.h>
#include <gtest/gtest.h>
#include <string>

#include "phoenix_bridge/write_conversions.h"

TEST(WriteConversionTests, TestStringMsg)
{
    std::string test_data = "test_msg";
    std::string test_instance_path = "test_path";

    std_msgs::String msg_to_test;
    msg_to_test.data = test_data;

    IDataAccessServiceWriteRequest request;
    ::Arp::Plc::Gds::Services::Grpc::WriteItem * grpc_object = request.add_data();
    grpc_object->set_portname(test_instance_path);
    grpc_object->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    conversions::packWriteItem(grpc_object, msg_to_test);

    std::string debugstr;
    google::protobuf::TextFormat::PrintToString(*grpc_object, &debugstr);

    ASSERT_TRUE(debugstr.find("PortName: \"test_path\"") !=std::string::npos);
    /// Counted by first manually verifying debugstr
    ASSERT_EQ(static_cast<int>(debugstr.find("StringValue: \"test_msg\"")), 121); // string.data
}

TEST(WriteConversionTests, TestDoubleMsg)
{
    std::string test_instance_path = "test_path";
    std_msgs::Float64 msg_to_test;
    msg_to_test.data = 12345.09876;

    IDataAccessServiceWriteRequest request;
    ::Arp::Plc::Gds::Services::Grpc::WriteItem * grpc_object = request.add_data();
    grpc_object->set_portname(test_instance_path);
    grpc_object->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    conversions::packWriteItem(grpc_object, msg_to_test);

    std::string debugstr;
    google::protobuf::TextFormat::PrintToString(*grpc_object, &debugstr);

    EXPECT_TRUE(debugstr.find("PortName: \"test_path\"") !=std::string::npos);
    /// Counted by first manually verifying debugstr
    EXPECT_EQ(static_cast<int>(debugstr.find("DoubleValue: 12345.09876")), 121);
}

TEST(WriteConversionTests, TestIntMsg)
{
    std::string test_instance_path = "test_path";
    std_msgs::Int64 msg_to_test;
    msg_to_test.data = 1234509876;

    IDataAccessServiceWriteRequest request;
    ::Arp::Plc::Gds::Services::Grpc::WriteItem * grpc_object = request.add_data();
    grpc_object->set_portname(test_instance_path);
    grpc_object->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Int64);

    conversions::packWriteItem(grpc_object, msg_to_test);

    std::string debugstr;
    google::protobuf::TextFormat::PrintToString(*grpc_object, &debugstr);

    EXPECT_TRUE(debugstr.find("PortName: \"test_path\"") !=std::string::npos);
    /// Counted by first manually verifying debugstr
    EXPECT_EQ(static_cast<int>(debugstr.find("Int64Value: 1234509876")), 120);
}

TEST(WriteConversionTests, TestTwistMsg)
{
  std::string test_instance_path = "test_path";

  geometry_msgs::Twist msg_to_test;
  msg_to_test.linear.x = 1.11;
  msg_to_test.linear.y = 2.22;
  msg_to_test.linear.z = 3.33;
  msg_to_test.angular.x = 4.44;
  msg_to_test.angular.y = 5.55;
  msg_to_test.angular.z = 6.66;

  IDataAccessServiceWriteRequest request;
  ::Arp::Plc::Gds::Services::Grpc::WriteItem * grpc_object = request.add_data();
  grpc_object->set_portname(test_instance_path);
  grpc_object->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  conversions::packWriteItem(grpc_object, msg_to_test);

  std::string debugstr;
  google::protobuf::TextFormat::PrintToString(*grpc_object, &debugstr);

  ASSERT_TRUE(debugstr.find("PortName: \"test_path\"") !=std::string::npos);
  /// Counted by first manually verifying debugstr
  ASSERT_EQ(static_cast<int>(debugstr.find("DoubleValue: 1.11")), 200);  // linear.x
  ASSERT_EQ(static_cast<int>(debugstr.find("DoubleValue: 2.22")), 293);  // linear.y
  ASSERT_EQ(static_cast<int>(debugstr.find("DoubleValue: 3.33")), 386);  // linear.z
  ASSERT_EQ(static_cast<int>(debugstr.find("DoubleValue: 4.44")), 560);  // angular.x
  ASSERT_EQ(static_cast<int>(debugstr.find("DoubleValue: 5.55")), 653);  // angular.y
  ASSERT_EQ(static_cast<int>(debugstr.find("DoubleValue: 6.66")), 746);  // angular.z
}

TEST(WriteConversionTests, TestHeaderMsg)
{
  std::string test_instance_path = "test_path";

  std_msgs::Header msg_to_test;
  msg_to_test.stamp.sec = 1658485862;
  msg_to_test.stamp.nsec = 602742553;
  msg_to_test.frame_id = "header_frame_id";

  IDataAccessServiceWriteRequest request;
  ::Arp::Plc::Gds::Services::Grpc::WriteItem * grpc_object = request.add_data();
  grpc_object->set_portname(test_instance_path);
  grpc_object->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  conversions::packWriteItem(grpc_object, msg_to_test);

  std::string debugstr;
  google::protobuf::TextFormat::PrintToString(*grpc_object, &debugstr);

  ASSERT_TRUE(debugstr.find("PortName: \"test_path\"") !=std::string::npos);
  /// Counted by first manually verifying debugstr

  // Time is handled as double of `sec.nsec` but some precision is lost
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 1658485862.6027")),-1);
  ASSERT_NE(static_cast<int>(debugstr.find("StringValue: \"header_frame_id\"")), -1);
}

TEST(WriteConversionTests, TestOdomMsg)
{
  std::string test_instance_path = "test_path";

  nav_msgs::Odometry msg_to_test;
  msg_to_test.header.stamp.sec = 1658485862;
  msg_to_test.header.stamp.nsec = 602742553;
  msg_to_test.header.frame_id = "header_frame_id";
  msg_to_test.child_frame_id = "child_frame_id";
  msg_to_test.pose.pose.position.x = 1.11;
  msg_to_test.pose.pose.position.y = 2.22;
  msg_to_test.pose.pose.position.z = 3.33;
  msg_to_test.pose.pose.orientation.x = 4.44;
  msg_to_test.pose.pose.orientation.y = 5.55;
  msg_to_test.pose.pose.orientation.z = 6.66;
  msg_to_test.pose.pose.orientation.w = 7.77;
  msg_to_test.pose.covariance.at(5) = 8.88;
  msg_to_test.twist.twist.linear.x = 9.99;
  msg_to_test.twist.twist.linear.y = 10.1010;
  msg_to_test.twist.twist.linear.z = 11.1111;
  msg_to_test.twist.twist.angular.x = 12.1212;
  msg_to_test.twist.twist.angular.y = 13.1313;
  msg_to_test.twist.twist.angular.z = 14.1414;
  msg_to_test.twist.covariance.at(7) = 15.1515;

  IDataAccessServiceWriteRequest request;
  ::Arp::Plc::Gds::Services::Grpc::WriteItem * grpc_object = request.add_data();
  grpc_object->set_portname(test_instance_path);
  grpc_object->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

  conversions::packWriteItem(grpc_object, msg_to_test);

  std::string debugstr;
  google::protobuf::TextFormat::PrintToString(*grpc_object, &debugstr);

  ASSERT_TRUE(debugstr.find("PortName: \"test_path\"") !=std::string::npos);
  /// Counted by first manually verifying debugstr

  // Time is handled as double of `sec.nsec` but some precision is lost
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 1658485862.6027")),-1);
  ASSERT_NE(static_cast<int>(debugstr.find("StringValue: \"header_frame_id\"")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("StringValue: \"child_frame_id\"")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 1.11")),-1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 2.22")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 3.33")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 4.44")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 5.55")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 6.66")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 7.77")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 8.88")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 9.99")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 10.101")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 11.11")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 12.1212")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 13.1313")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 14.1414")), -1);
  ASSERT_NE(static_cast<int>(debugstr.find("DoubleValue: 15.1515")), -1);

}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}