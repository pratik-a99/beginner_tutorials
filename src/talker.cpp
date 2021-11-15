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
 * @file  talker.cpp
 * @brief A publisher and service tutorial C++ node, which includes a transform
 * function which broadcasts the position and orientation of a coordinate frame
 * @author Pratik Acharya
 */

#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "beginner_tutorials/serviceType.h"
#include "beginner_tutorials/talker.h"

/**
 * @fn bool changeOutput(beginner_tutorials::serviceType::Request&, beginner_tutorials::serviceType::Response&)
 * @brief A service callback function which changes the output message of the talker
 *
 * @param req
 * @param resp
 * @return true or false
 */
bool changeOutput(beginner_tutorials::serviceType::Request &req,
                  beginner_tutorials::serviceType::Response &resp) {
  ROS_DEBUG_STREAM("The service has been called successully");  // Debug log

  if (req.inputString.empty()) {
    // Error log
    ROS_ERROR_STREAM("An empty string was given as service input");
    // Fatal log
    ROS_FATAL_STREAM("The service will be stopped due to incorrect input");
    return false;
  } else {
    ROS_WARN_STREAM(
        "The publisher message will be changed to \"" << req.inputString
            << "\"");                 // Warn log
    message = req.inputString;
    resp.outputString = req.inputString;
    return true;
  }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);

  int countRate;

  n.getParam("/count_rate", countRate);

  /**
   * Condition to check the given param for count rate
   */
  if (countRate < 1) {
    ROS_FATAL_STREAM("The rate of counting cannot be negative or zero");
    ROS_INFO_STREAM("Count rate set to default value of 10");
    countRate = 10;
  } else if (countRate > 10 && countRate <= 20) {
    ROS_WARN_STREAM("Count rate too high");
  } else if (countRate > 20) {
    ROS_FATAL_STREAM("Count rate higher than permissable");
    ROS_INFO_STREAM("Count rate set to default value of 10");
    countRate = 10;
  } else {
    ROS_INFO_STREAM("Enjoy the counting at your rate set at " << countRate);
  }

  ros::Rate loop_rate(countRate);

  ros::ServiceServer service = n.advertiseService("outputService",
                                                  &changeOutput);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  auto count = 0;

  /**
   * Loop to publish the message with defined rate
   */
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << message << count;
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    /**
     * Creating a broadcaster for the transform function
     */
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Setting the origin for the coordinate frame
    transform.setOrigin(tf::Vector3(count, 2.0, 3.0));
    tf::Quaternion q;
    q.setRPY(1, 2, count);
    transform.setRotation(q);

    // Broadcasting the transformation
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    // Publishing message to chatter topic
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
