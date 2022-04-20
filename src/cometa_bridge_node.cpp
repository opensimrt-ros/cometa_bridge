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

#include <ros/console.h>

void mySigintHandler(int sig)
{
    // Do custom action, like publishing stop msg
    ros::shutdown();
}

#define PI 3.141592 
#define MAX_DELAY 0.1
#define GRAVITY 9.80665

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
//ACC* is a linear acceleration estimation from ngimu, really useless for us and I kinda want to remove it. 


geometry_msgs::Vector3 convert_gyro_to_rad(geometry_msgs::Vector3 gyro_degree_per_second)
{
	geometry_msgs::Vector3 out_gyro;
	out_gyro.x = gyro_degree_per_second.x/180.0*PI;
	out_gyro.y = gyro_degree_per_second.y/180.0*PI;
	out_gyro.z = gyro_degree_per_second.z/180.0*PI;
	return out_gyro;
}

geometry_msgs::Vector3 convert_acc_g_to_ms(geometry_msgs::Vector3 acc_g)
{
	geometry_msgs::Vector3 out_acc;
	out_acc.x = acc_g.x*GRAVITY;
	out_acc.y = acc_g.y*GRAVITY;
	out_acc.y = acc_g.y*GRAVITY;
	return out_acc;	
}

std::deque<sensor_msgs::Imu> convert_text (std::deque<double>& vec)
{
	//using enum CometaImuOrder;
	std::deque<sensor_msgs::Imu> imus;

	double time = vec[0];
	vec.pop_front();
	ROS_DEBUG_STREAM("Time: " << time);	
	
	ROS_DEBUG_STREAM("==================== vec size " << vec.size());
	
	for (int i=0; i < vec.size()/18; i++)
	{
		ROS_DEBUG_STREAM("Adding imu index i: " << i );
		sensor_msgs::Imu thisImu;

		
		//it needs a header
		std_msgs::Header h;
		//TODO: stamp it!
		//now it is more complicated than this. I think maybe I should read the time from each packet and use that as a stamp
		//h.frame_id = "imu_frame"+std::to_string(i); //not sure what to put here.
		h.frame_id = "map";
		double now = ros::Time::now().toSec();
		if (now - time > MAX_DELAY)
			ROS_WARN_STREAM("The time delay difference between acquired data is greater than " << MAX_DELAY << " publication time: " << time << " Now is: " << now );
		//ros::Time rt(now);
		//ros::Time rt(time);
		//h.stamp = ros::Time::Time(time);
		//h.stamp = rt;
		h.stamp = ros::Time::now();

		//let's put the original quaternion here as well
		geometry_msgs::Quaternion q;
		q.x = vec[i*18+QUATERNIONX];
		q.y = vec[i*18+QUATERNIONY];
		q.z = vec[i*18+QUATERNIONZ];
		q.w = vec[i*18+QUATERNIONW];

		ROS_DEBUG_STREAM("Quaternion read: " <<q);
		//then angular velocity (gyro), ngimu is being used as standard here, and it has degrees/s as unit https://x-io.co.uk/downloads/NGIMU-User-Manual-v1.6.pdf
		geometry_msgs::Vector3 gyro;

		gyro.x = vec[i*18+GYROX];
		gyro.y = vec[i*18+GYROY];
		gyro.z = vec[i*18+GYROZ];
		ROS_DEBUG_STREAM("Gyro read: " << gyro);


		//and acceleration
		geometry_msgs::Vector3 acc;
		acc.x = vec[i*18+AX]; 
		acc.y = vec[i*18+AY];
		acc.z = vec[i*18+AZ];
		ROS_DEBUG_STREAM("Acc read:" << acc);
		//there is also something called covariance. idk why i need it and where to get it, so it will remain unset.
		
		thisImu.header = h;

		geometry_msgs::Vector3 cgyro = convert_gyro_to_rad(gyro);
		thisImu.angular_velocity = cgyro;
		ROS_DEBUG_STREAM("Gyro converted: " << cgyro);

		geometry_msgs::Vector3 cacc = convert_acc_g_to_ms(acc);
		ROS_DEBUG_STREAM("Acc converted:" << cacc);
		thisImu.linear_acceleration = cacc;
		//thisImu.angular_velocity = (gyro);
		//thisImu.linear_acceleration = (acc);
		thisImu.orientation = q;

		imus.push_back(thisImu);
		ROS_DEBUG_STREAM("Finished parsing imu index i: " << i );

	}
	ROS_DEBUG_STREAM("Do I return or crash?");
	return imus;
}	


int main(int argc, char **argv)
{
    /*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }*/


  signal(SIGINT, mySigintHandler);
  ros::init(argc, argv, "imu_bridge");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("imu_driver_string", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  int MAXIMUS = 10;
  nh.getParam("num_imus", MAXIMUS);
  ROS_INFO_STREAM("NUM imus " << MAXIMUS );
  std::deque<ros::Publisher> ImuPubs;
  for (int i= 0; i < MAXIMUS; i++)
  {
	  ROS_INFO_STREAM("Iterating publishers: " << i);
	  ImuPubs.push_back(n.advertise<sensor_msgs::Imu>("imu"+std::to_string(i)+"/data_raw", 1000));  
  }
  SimpleServer server;
  ROS_INFO("Started cometa bridge listener.");
  while (ros::ok())
  {
    if (server.receive())
    {
	    ROS_DEBUG_STREAM("SimpleServer received okay.");
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
    else
    {
	ROS_WARN_STREAM("No message from bridge.");
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
