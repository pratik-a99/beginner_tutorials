#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/serviceType.h"

std::shared_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, service_test1)
{
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials::serviceType>(
      "outputService");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

TEST(TESTSuite, service_test2)
{
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials::serviceType>(
      "outputService");

  beginner_tutorials::serviceType srv;
  srv.request.inputString = "Service Test :";
  client.call(srv);

  EXPECT_EQ(srv.response.outputString, srv.request.inputString);
}

TEST(TESTSuite, master_test){
  EXPECT_TRUE(ros::master::check());
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "ros_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

