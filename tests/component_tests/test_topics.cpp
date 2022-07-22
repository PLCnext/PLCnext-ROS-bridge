#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <algorithm>

class TestTopics : public testing::Test
{
public:
    TestTopics() {}
};

/**
 * @brief Check if node and topics are created as expected
 *
 */
TEST_F(TestTopics, check_topics)
{
    ros::Time start = ros::Time::now();
    std::vector<std::string> nodes;
    while (ros::ok())
    {
        ros::master::getNodes(nodes);
        if (nodes.size() > 1)
            break;
        // Timeout of 5 secods for tested node to start
        ASSERT_LE(ros::Time::now() - start, ros::Duration(5))<< "Timed out waiting for node to spawn";
    }

    //  Test that the expected node is created
    ASSERT_THAT(nodes, testing::Contains("/phoenix_bridge")) << "Node /phoenix_bridge not found in list";

    ros::master::V_TopicInfo topic_infos;
    while(topic_infos.size() < 7)  // Wait until all 7 expected topics spawn
    {
        ros::master::getTopics(topic_infos);
    }
    std::vector<std::string> topics, datatypes;

    for (auto topic : topic_infos)
    {
        topics.push_back(topic.name);
        datatypes.push_back(topic.datatype);
    }

    // Can only check publishers, since getTopics() returns only publishers
    ASSERT_THAT(topics, testing::Contains("/pub_odom_1"));
    ASSERT_THAT(topics, testing::Contains("/pub_odom_2"));
    ASSERT_THAT(topics, testing::Contains("/pub_twist_1"));
    ASSERT_THAT(topics, testing::Contains("/pub_twist_2"));
    ASSERT_THAT(topics, testing::Contains("/pub_string_1"));
    ASSERT_THAT(topics, testing::Contains("/pub_string_2"));

    ASSERT_THAT(datatypes, testing::Contains("nav_msgs/Odometry"));
    ASSERT_THAT(datatypes, testing::Contains("geometry_msgs/Twist"));
    ASSERT_THAT(datatypes, testing::Contains("std_msgs/String"));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_topics");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
