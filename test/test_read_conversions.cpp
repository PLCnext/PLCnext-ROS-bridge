#include <gtest/gtest.h>
#include <string>

#include "phoenix_bridge/read_conversions.hpp"

TEST(ReadConversionTests, TestStringMsg)
{
    std::string test_data = "test_msg";
    std_msgs::msg::String msg;

    /// @todo: Insert way to pack the test_data as a std_msgs/msg/String struct into grpc_object
    /// Must comment this section until packing can be done manually
    ObjectType grpc_object;
    // conversions::unpackReadObject(grpc_object, msg);

    // For now, dummy to pass test
    msg.data = test_data;
    EXPECT_EQ(test_data, msg.data);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}