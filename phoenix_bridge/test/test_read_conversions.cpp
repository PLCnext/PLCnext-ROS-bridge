#include <google/protobuf/text_format.h>
#include <gtest/gtest.h>
#include <string>

#include "phoenix_bridge/read_conversions.hpp"

using ::Arp::Type::Grpc::CoreType;

TEST(ReadConversionTests, TestStringMsg)
{
    std::string test_data = "test_msg";
    std_msgs::msg::String msg;

    IDataAccessServiceReadResponse reply;
    ::Arp::Plc::Gds::Services::Grpc::ReadItem* read_item = reply.add__returnvalue();

    ::Arp::Type::Grpc::ObjectType* data = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    data->set_typecode(::Arp::Type::Grpc::CoreType::CT_String);
    data->set_stringvalue(test_data);

    conversions::unpackReadObject(read_item->value(), msg);

    EXPECT_EQ(test_data, msg.data);
}

TEST(ReadConversionTests, TestDoubleMsg)
{
    double test_data = 12345.09876;
    std_msgs::msg::Float64 msg;

    IDataAccessServiceReadResponse reply;
    ::Arp::Plc::Gds::Services::Grpc::ReadItem* read_item = reply.add__returnvalue();

    ::Arp::Type::Grpc::ObjectType* data = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    data->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    data->set_doublevalue(test_data);

    conversions::unpackReadObject(read_item->value(), msg);

    EXPECT_EQ(test_data, msg.data);
}

TEST(ReadConversionTests, TestIntMsg)
{
    int64_t test_data = 1234509876;
    std_msgs::msg::Int64 msg;

    IDataAccessServiceReadResponse reply;
    ::Arp::Plc::Gds::Services::Grpc::ReadItem* read_item = reply.add__returnvalue();

    ::Arp::Type::Grpc::ObjectType* data = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    data->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    data->set_int64value(test_data);

    conversions::unpackReadObject(read_item->value(), msg);

    EXPECT_EQ(test_data, msg.data);
}

TEST(ReadConversionTests, TestTwistMsg)
{
    geometry_msgs::msg::Twist twist_msg;

    IDataAccessServiceReadResponse reply;
    ::Arp::Plc::Gds::Services::Grpc::ReadItem* read_item = reply.add__returnvalue();

    read_item->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* linear_1 = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    linear_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* linear_x = linear_1->mutable_structvalue()->add_structelements();
    linear_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    linear_x->set_doublevalue(1.11);

    ::Arp::Type::Grpc::ObjectType* linear_y = linear_1->mutable_structvalue()->add_structelements();
    linear_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    linear_y->set_doublevalue(2.22);

    ::Arp::Type::Grpc::ObjectType* linear_z = linear_1->mutable_structvalue()->add_structelements();
    linear_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    linear_z->set_doublevalue(3.33);

    ::Arp::Type::Grpc::ObjectType* angular_1 = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    angular_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* angular_x = angular_1->mutable_structvalue()->add_structelements();
    angular_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    angular_x->set_doublevalue(4.44);

    ::Arp::Type::Grpc::ObjectType* angular_y = angular_1->mutable_structvalue()->add_structelements();
    angular_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    angular_y->set_doublevalue(5.55);

    ::Arp::Type::Grpc::ObjectType* angular_z = angular_1->mutable_structvalue()->add_structelements();
    angular_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    angular_z->set_doublevalue(6.66);

    conversions::unpackReadObject(read_item->value(), twist_msg);

    EXPECT_EQ(twist_msg.linear.x, 1.11);  // x
    EXPECT_EQ(twist_msg.linear.y, 2.22);  // y
    EXPECT_EQ(twist_msg.linear.z, 3.33);  // z
    EXPECT_EQ(twist_msg.angular.x, 4.44);  // x
    EXPECT_EQ(twist_msg.angular.y, 5.55);  // y
    EXPECT_EQ(twist_msg.angular.z, 6.66);  // z
}

TEST(ReadConversionTests, TestHeaderMsg)
{
    std_msgs::msg::Header header_msg;

    IDataAccessServiceReadResponse reply;

    ::Arp::Plc::Gds::Services::Grpc::ReadItem* read_item = reply.add__returnvalue();

    read_item->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* stamp_1 = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    stamp_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* stamp_sec = stamp_1->mutable_structvalue()->add_structelements();
    stamp_sec->set_typecode(::Arp::Type::Grpc::CoreType::CT_Int32);
    stamp_sec->set_int32value(1658485862);

    ::Arp::Type::Grpc::ObjectType* stamp_nanosec = stamp_1->mutable_structvalue()->add_structelements();
    stamp_nanosec->set_typecode(::Arp::Type::Grpc::CoreType::CT_Uint32);
    stamp_nanosec->set_uint32value(602742553);

    ::Arp::Type::Grpc::ObjectType* frame_id = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    frame_id->set_typecode(::Arp::Type::Grpc::CoreType::CT_String);
    frame_id->set_stringvalue("frame_id");

    ObjectType test_grpc_object = reply._returnvalue(0).value();

    conversions::unpackReadObject(test_grpc_object, header_msg);

    EXPECT_EQ(header_msg.stamp.sec, 1658485862);
    EXPECT_EQ((header_msg.stamp.nanosec), 602742553U);
    EXPECT_EQ(header_msg.frame_id, "frame_id");
}



TEST(ReadConversionTests, TestOdomMsg)
{
    nav_msgs::msg::Odometry odom_msg;

    IDataAccessServiceReadResponse reply;

    ::Arp::Plc::Gds::Services::Grpc::ReadItem* read_item = reply.add__returnvalue();

    read_item->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* header_1 = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    header_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* stamp_2 = header_1->mutable_structvalue()->add_structelements();
    stamp_2->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* header_stamp_sec = stamp_2->mutable_structvalue()->add_structelements();
    header_stamp_sec->set_typecode(::Arp::Type::Grpc::CoreType::CT_Int32);
    header_stamp_sec->set_int32value(1658485862);

    ::Arp::Type::Grpc::ObjectType* header_stamp_nanosec = stamp_2->mutable_structvalue()->add_structelements();
    header_stamp_nanosec->set_typecode(::Arp::Type::Grpc::CoreType::CT_Uint32);
    header_stamp_nanosec->set_uint32value(602742553);

    ::Arp::Type::Grpc::ObjectType* header_frame_id = header_1->mutable_structvalue()->add_structelements();
    header_frame_id->set_typecode(::Arp::Type::Grpc::CoreType::CT_String);
    header_frame_id->set_stringvalue("header_frame_id");

    ::Arp::Type::Grpc::ObjectType* child_frame_id = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    child_frame_id->set_typecode(::Arp::Type::Grpc::CoreType::CT_String);
    child_frame_id->set_stringvalue("child_frame_id");

    ::Arp::Type::Grpc::ObjectType* pose_1 = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    pose_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* pose_2 = pose_1->mutable_structvalue()->add_structelements();
    pose_2->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* position_3 = pose_2->mutable_structvalue()->add_structelements();
    position_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* pose_pose_position_x = position_3->mutable_structvalue()->add_structelements();
    pose_pose_position_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    pose_pose_position_x->set_doublevalue(1.11);

    ::Arp::Type::Grpc::ObjectType* pose_pose_position_y = position_3->mutable_structvalue()->add_structelements();
    pose_pose_position_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    pose_pose_position_y->set_doublevalue(2.22);

    ::Arp::Type::Grpc::ObjectType* pose_pose_position_z = position_3->mutable_structvalue()->add_structelements();
    pose_pose_position_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    pose_pose_position_z->set_doublevalue(3.33);

    ::Arp::Type::Grpc::ObjectType* orientation_3 = pose_2->mutable_structvalue()->add_structelements();
    orientation_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_x = orientation_3->mutable_structvalue()->add_structelements();
    pose_pose_orientation_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    pose_pose_orientation_x->set_doublevalue(4.44);

    ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_y = orientation_3->mutable_structvalue()->add_structelements();
    pose_pose_orientation_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    pose_pose_orientation_y->set_doublevalue(5.55);

    ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_z = orientation_3->mutable_structvalue()->add_structelements();
    pose_pose_orientation_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    pose_pose_orientation_z->set_doublevalue(6.66);

    ::Arp::Type::Grpc::ObjectType* pose_pose_orientation_w = orientation_3->mutable_structvalue()->add_structelements();
    pose_pose_orientation_w->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    pose_pose_orientation_w->set_doublevalue(7.77);

    ::Arp::Type::Grpc::ObjectType* pose_covariance = pose_1->mutable_structvalue()->add_structelements();
    pose_covariance->set_typecode(::Arp::Type::Grpc::CoreType::CT_Array);
    ::Arp::Type::Grpc::TypeArray* pose_covariance_array = pose_covariance->mutable_arrayvalue();
    for (int i = 0; i < 36; i++)
    {
      ObjectType* elem = pose_covariance_array->add_arrayelements();
      elem->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
      elem->set_doublevalue(i);
    }

    ::Arp::Type::Grpc::ObjectType* twist_1 = read_item->mutable_value()->mutable_structvalue()->add_structelements();
    twist_1->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* twist_2 = twist_1->mutable_structvalue()->add_structelements();
    twist_2->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* linear_3 = twist_2->mutable_structvalue()->add_structelements();
    linear_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* twist_twist_linear_x = linear_3->mutable_structvalue()->add_structelements();
    twist_twist_linear_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    twist_twist_linear_x->set_doublevalue(8.88);

    ::Arp::Type::Grpc::ObjectType* twist_twist_linear_y = linear_3->mutable_structvalue()->add_structelements();
    twist_twist_linear_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    twist_twist_linear_y->set_doublevalue(9.99);

    ::Arp::Type::Grpc::ObjectType* twist_twist_linear_z = linear_3->mutable_structvalue()->add_structelements();
    twist_twist_linear_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    twist_twist_linear_z->set_doublevalue(10.101);

    ::Arp::Type::Grpc::ObjectType* angular_3 = twist_2->mutable_structvalue()->add_structelements();
    angular_3->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);

    ::Arp::Type::Grpc::ObjectType* twist_twist_angular_x = angular_3->mutable_structvalue()->add_structelements();
    twist_twist_angular_x->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    twist_twist_angular_x->set_doublevalue(11.1111);

    ::Arp::Type::Grpc::ObjectType* twist_twist_angular_y = angular_3->mutable_structvalue()->add_structelements();
    twist_twist_angular_y->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    twist_twist_angular_y->set_doublevalue(12.1212);

    ::Arp::Type::Grpc::ObjectType* twist_twist_angular_z = angular_3->mutable_structvalue()->add_structelements();
    twist_twist_angular_z->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
    twist_twist_angular_z->set_doublevalue(13.1313);

    ::Arp::Type::Grpc::ObjectType* twist_covariance = twist_1->mutable_structvalue()->add_structelements();
    twist_covariance->set_typecode(::Arp::Type::Grpc::CoreType::CT_Array);
    ::Arp::Type::Grpc::TypeArray* twist_covariance_array = twist_covariance->mutable_arrayvalue();
    for (int i = 0; i < 36; i++)
    {
      ObjectType* elem = twist_covariance_array->add_arrayelements();
      elem->set_typecode(::Arp::Type::Grpc::CoreType::CT_Real64);
      elem->set_doublevalue(i);
    }

    ObjectType test_grpc_object = reply._returnvalue(0).value();

    conversions::unpackReadObject(test_grpc_object, odom_msg);

    EXPECT_EQ(odom_msg.header.stamp.sec, 1658485862);
    EXPECT_EQ((odom_msg.header.stamp.nanosec), 602742553U);
    EXPECT_EQ(odom_msg.header.frame_id, "header_frame_id");
    EXPECT_EQ(odom_msg.child_frame_id, "child_frame_id");
    EXPECT_EQ(odom_msg.pose.pose.position.x , 1.11);
    EXPECT_EQ(odom_msg.pose.pose.position.y , 2.22);
    EXPECT_EQ(odom_msg.pose.pose.position.z , 3.33);
    EXPECT_EQ(odom_msg.pose.pose.orientation.x , 4.44);
    EXPECT_EQ(odom_msg.pose.pose.orientation.y , 5.55);
    EXPECT_EQ(odom_msg.pose.pose.orientation.z , 6.66);
    EXPECT_EQ(odom_msg.pose.pose.orientation.w , 7.77);
    for (int i = 0; i < 36; i++)
    {
      EXPECT_EQ(odom_msg.pose.covariance[i], i);
      EXPECT_EQ(odom_msg.twist.covariance[i], i);
    }
    EXPECT_EQ(odom_msg.twist.twist.linear.x , 8.88);
    EXPECT_EQ(odom_msg.twist.twist.linear.y , 9.99);
    EXPECT_EQ(odom_msg.twist.twist.linear.z , 10.101);
    EXPECT_EQ(odom_msg.twist.twist.angular.x , 11.1111);
    EXPECT_EQ(odom_msg.twist.twist.angular.y , 12.1212);
    EXPECT_EQ(odom_msg.twist.twist.angular.z , 13.1313);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
