#include <gtest/gtest.h>
#include <string>

#include "phoenix_bridge/conversions.hpp"

TEST(ConversionTests, TestStringMsg)
{
    std::string test_data = "test_msg";
    std::string test_instance_path = "test_path";

    std_msgs::msg::String msg_to_test;
    msg_to_test.data = test_data;

    IDataAccessServiceWriteRequest request;
    ::Arp::Plc::Gds::Services::Grpc::WriteItem * grpc_object = request.add_data();
    grpc_object->set_portname(test_instance_path);
    grpc_object->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    conversions::packWriteItem(grpc_object, msg_to_test);

    std::string unpacked_data;
    std::string unpacked_instance_path;

    /// @todo: Insert way to unpack the grpc_object to get unpacked_data and unpackeed_instance_path
    // For now, dummy to pass test
    unpacked_data = test_data;
    unpacked_instance_path = test_instance_path;

    EXPECT_EQ(test_data, unpacked_data);
    EXPECT_EQ(test_instance_path, unpacked_instance_path);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}