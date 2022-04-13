#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <csignal>
#include "SimpleServer.h"

void mySigintHandler(int sig)
{
    // Do custom action, like publishing stop msg
    ros::shutdown();
}

int main(int argc, char **argv)
{
  signal(SIGINT, mySigintHandler);
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  SimpleServer server();
  while (ros::ok())
  {
    if (server.receive())
    {
	    std_msgs::String msg;
	    std::stringstream ss;
	    ss << server.buffer << count;
	    msg.data = ss.str();
	    ROS_INFO("%s", msg.data.c_str());
	    chatter_pub.publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
