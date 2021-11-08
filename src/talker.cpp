/**
 * @copyright  MIT License (c) 2021 Pratik Acharya
 * @file  talker.cpp
 * @brief A publisher and service tutorial C++ node
 * @author Pratik Acharya
 */

#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/serviceType.h"

std::string message = "Yay! I can now count till ";

bool changeOutput(beginner_tutorials::serviceType::Request &req,
                  beginner_tutorials::serviceType::Response &resp) {
  ROS_DEBUG_STREAM("The service has been called successully");

  if (req.inputString.empty()) {
    ROS_ERROR_STREAM("An empty string was given as service input");
    ROS_FATAL_STREAM("The service will be stopped due to incorrect input");
    return false;
  } else {
    ROS_WARN_STREAM(
        "The publisher message will be changed to \"" << req.inputString
            << "\"");
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
  if(countRate < 1){
    ROS_FATAL_STREAM("The rate of counting cannot be negative or zero");
    ROS_INFO_STREAM("Count rate set to default value of 10");
    countRate = 10;
  }
  else if (countRate > 10 && countRate <= 20) {
    ROS_WARN_STREAM("Count rate too high");
  }
  else if (countRate > 20) {
    ROS_FATAL_STREAM("Count rate higher than permissable");
    ROS_INFO_STREAM("Count rate set to default value of 10");
    countRate = 10;
  }
  else {
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
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << message << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
