/**
 * Copyright (C) MIT.
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @copyright  MIT License (c) 2021 Pratik Acharya
 * @file  ros_test.cpp
 * @brief A rostest example to demonstate Level 2 testing framework
 * @author Pratik Acharya
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/serviceType.h"

// Creating a NodeHandle Pointer
std::shared_ptr<ros::NodeHandle> nh;

/**
 * @brief Test to check the existence of services declared in 
 * talker.cpp file
 */
TEST(TESTSuite, service_test1) {
  ros::ServiceClient client =
      nh->serviceClient<beginner_tutorials::serviceType>(
      "outputService");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

/**
 * @brief Test to check if the service is working as expected
 */
TEST(TESTSuite, service_test2) {
  ros::ServiceClient client =
      nh->serviceClient<beginner_tutorials::serviceType>(
      "outputService");

  beginner_tutorials::serviceType srv;
  srv.request.inputString = "Service Test :";
  client.call(srv);

  EXPECT_EQ(srv.response.outputString, srv.request.inputString);
}

/**
 * @brief Test to check if roscore is running
 */
TEST(TESTSuite, master_test) {
  EXPECT_TRUE(ros::master::check());
}

int main(int argc,
         char **argv) {
  ros::init(argc, argv, "ros_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

