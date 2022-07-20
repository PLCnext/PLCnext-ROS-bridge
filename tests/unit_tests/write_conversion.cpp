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


TEST(WriteConversionTests, TestOdomMsg)
{
  std::string test_instance_path = "test_path";

  nav_msgs::Odometry msg_to_test;
  msg_to_test.header.stamp.sec = 111;
  msg_to_test.header.stamp.nsec = 222;
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
  /// @TODO: Update this test properly once time setting is figured out. Use ASSERT_EQ with the right numbers
  // ASSERT_EQ(static_cast<int>(debugstr.find("Int32Value: 111")) ,290);
  // ASSERT_EQ(static_cast<int>(debugstr.find("Uint32Value: 222")) , 397);
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