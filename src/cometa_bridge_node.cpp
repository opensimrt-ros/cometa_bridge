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

/*
 * a = """
thorax_q1    thorax_q2    thorax_q3    thorax_q4    thorax_ax    thorax_ay    thorax_az    thorax_gx    thorax_gy    thorax_gz    thorax_mx    thorax_my    thorax_mz    thorax_barometer    thorax_linAcc_x    thorax_linAcc_y    thorax_linAcc_z    thorax_altitude 
"""
 *
 *
 * */
//print("\n".join(["COMETA_"+j.split("thorax_")[-1].upper()+"="+str(i)+"," for i,j in enumerate(a.split())]))


enum CometaImuOrder {
	COMETA_Q1=0,
	COMETA_Q2=1,
	COMETA_Q3=2,
	COMETA_Q4=3,
	COMETA_AX=4,
	COMETA_AY=5,
	COMETA_AZ=6,
	COMETA_GX=7,
	COMETA_GY=8,
	COMETA_GZ=9,
	COMETA_MX=10,
	COMETA_MY=11,
	COMETA_MZ=12,
	COMETA_BAROMETER=13,
	COMETA_LINACC_X=14,
	COMETA_LINACC_Y=15,
	COMETA_LINACC_Z=16,
	COMETA_ALTITUDE=17,
	/*	QUATERNIONX = 0,
	QUATERNIONY = 1,
	QUATERNIONZ = 2,
	QUATERNIONW = 3,
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
	ALTITUDE = 17*/
};
//ACC* is a linear acceleration estimation from ngimu, really useless for us and I kinda want to remove it. 


geometry_msgs::Vector3 convert_gyro_to_rad(geometry_msgs::Vector3 gyro_degree_per_second)
{
	geometry_msgs::Vector3 out_gyro;
	out_gyro.x = gyro_degree_per_second.x*PI/180.0;
	out_gyro.y = gyro_degree_per_second.y*PI/180.0;
	out_gyro.z = gyro_degree_per_second.z*PI/180.0;
	return out_gyro;
}

geometry_msgs::Vector3 convert_acc_g_to_ms(geometry_msgs::Vector3 acc_g)
{
	geometry_msgs::Vector3 out_acc;
	out_acc.x = acc_g.x*GRAVITY;
	out_acc.y = acc_g.y*GRAVITY;
	out_acc.z = acc_g.z*GRAVITY;
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
		q.w = vec[i*18+COMETA_Q1];
		q.x = vec[i*18+COMETA_Q2];
		q.y = vec[i*18+COMETA_Q3];
		q.z = vec[i*18+COMETA_Q4];

		ROS_DEBUG_STREAM("Quaternion read: " <<q);
		//then angular velocity (gyro), ngimu is being used as standard here, and it has degrees/s as unit https://x-io.co.uk/downloads/NGIMU-User-Manual-v1.6.pdf
		geometry_msgs::Vector3 gyro;

		gyro.x = vec[i*18+COMETA_GX];
		gyro.y = vec[i*18+COMETA_GY];
		gyro.z = vec[i*18+COMETA_GZ];
		ROS_DEBUG_STREAM("Gyro read: " << gyro);


		//and acceleration
		geometry_msgs::Vector3 acc;
		acc.x = vec[i*18+COMETA_AX]; 
		acc.y = vec[i*18+COMETA_AY];
		acc.z = vec[i*18+COMETA_AZ];
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
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }


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
