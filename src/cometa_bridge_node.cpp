#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <sstream>
#include <csignal>
#include <stdlib.h>
#include "SimpleServer.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <deque>

void mySigintHandler(int sig)
{
    // Do custom action, like publishing stop msg
    ros::shutdown();
}

enum CometaImuOrder {
	QUATERNIONW = 0,
	QUATERNIONX = 1,
	QUATERNIONY = 2,
	QUATERNIONZ = 3,
	AX = 4,
	AY = 5,
	AZ = 6,
	GYROX = 7,
	GYROY = 8,
	GYROZ = 9,
	MX = 10,
	MY = 11,
	MZ = 12,
	BARO = 13,
	ACCX = 14,
	ACCY = 15,
	ACCZ = 16,
	ALTITUDE = 17
};

std::deque<sensor_msgs::Imu> convert_text (std::deque<double>& vec)
{
	//using enum CometaImuOrder;
	std::deque<sensor_msgs::Imu> imus;

	double time = vec[0];
	vec.pop_front();
	ROS_DEBUG_STREAM("Time: " << time);	

	for (int i; i < vec.size()/18; i++)
	{
		ROS_DEBUG_STREAM("Adding imu index i: " << i );
		sensor_msgs::Imu thisImu;

		
		//it needs a header
		std_msgs::Header h;
		//TODO: stamp it!
		//h.frame_id = "imu_frame"+std::to_string(i); //not sure what to put here.
		h.frame_id = "map";
		h.stamp = ros::Time::now();
		//then angular velocity (gyro)
		geometry_msgs::Vector3 gyro;

		gyro.x = vec[i*18+GYROX];
		gyro.y = vec[i*18+GYROY];
		gyro.z = vec[i*18+GYROZ];

		//and linear acceleration
		geometry_msgs::Vector3 lin_acc;
		lin_acc.x = vec[i*18+ACCX]; //or AX ??? idk yet.
		lin_acc.y = vec[i*18+ACCY];
		lin_acc.z = vec[i*18+ACCZ];
		//there is also something called covariance. idk why i need it and where to get it, so it will remain unset.
		
		thisImu.header = h;
		thisImu.angular_velocity = gyro;
		thisImu.linear_acceleration = lin_acc;

		imus.push_back(thisImu);

	}

	return imus;
}	


int main(int argc, char **argv)
{
  signal(SIGINT, mySigintHandler);
  ros::init(argc, argv, "imu_bridge");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("imu_driver_string", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  int MAXIMUS = 10;
  nh.getParam("num_imus", MAXIMUS);
  std::deque<ros::Publisher> ImuPubs;
  for (int i= 0; i < MAXIMUS; i++)
  {
	  ROS_DEBUG_STREAM("Iterating publishers: " << i);
	  ImuPubs.push_back(n.advertise<sensor_msgs::Imu>("imu"+std::to_string(i)+"/data_raw", 1000));  
  }
  SimpleServer server;
  ROS_INFO("Started cometa bridge listener.");
  while (ros::ok())
  {
    if (server.receive())
    {
	    std::deque<sensor_msgs::Imu> I = convert_text(server.output);
	    ROS_DEBUG_STREAM("How many IMUs I found: " << I.size() << ". How many publishers I have: " << ImuPubs.size());
	    for (int i = 0; i < I.size(); i++)
	    {
		    //:combine(I, ImuPubs))

		    ROS_DEBUG_STREAM("i: " << i);
		    //ros::Publisher pub;
		    //sensor_msgs::Imu imumsg;
		    //boost::tie(imumsg, pub) = tup;
		    ImuPubs[i].publish(I[i]);
	    }
	    std_msgs::String msg;
	    std::stringstream ss;
	    ss << server.buffer << count;
	    msg.data = ss.str();
	    ROS_DEBUG("%s", msg.data.c_str());
	    chatter_pub.publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
