// Server side implementation of UDP client-server model

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "SimpleServer.h"

#include <iostream>
#include <sstream>
#include <iterator>
#include <deque>

#include <ros/ros.h>
#include <ros/console.h>
//#include <tf/transform_listener.h>

// Driver code
SimpleServer::SimpleServer (int PORT, int MAXLINE) {
		const char* logger_name = ROSCONSOLE_DEFAULT_NAME "." ;
		if( ros::console::set_logger_level(logger_name+ debugger_sink, ros::console::levels::Info) ) {
   			ros::console::notifyLoggerLevelsChanged();
		}

		//socklen_t sockfd;
		buffer = new char[MAXLINE];
		hello = "Hello from server";
		buffersize = MAXLINE;
		//struct sockaddr_in servaddr, cliaddr;
		
		// Creating socket file descriptor
		if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
			ROS_ERROR_STREAM_NAMED(debugger_sink, "socket creation failed.");
			perror("socket creation failed");
			exit(EXIT_FAILURE);
		}
		
		memset(&servaddr, 0, sizeof(servaddr));
		memset(&cliaddr, 0, sizeof(cliaddr));
		
		// Filling server information
		servaddr.sin_family = AF_INET; // IPv4
		servaddr.sin_addr.s_addr = INADDR_ANY;
		servaddr.sin_port = htons(PORT);
		
		// Bind the socket with the server address
		if ( bind(sockfd, (const struct sockaddr *)&servaddr,
				sizeof(servaddr)) < 0 )
		{
			ROS_ERROR_STREAM_NAMED(debugger_sink, "Bind error using port " << PORT );
			perror("bind failed");
			exit(EXIT_FAILURE);
		}
		
		//socklen_t len, n;

		len = sizeof(cliaddr); //len is value/resuslt

}

SimpleServer::~SimpleServer(void)
{
}

//std::vector<double> SimpleServer::receive()
bool SimpleServer::receive()
{
		/*n = recvfrom(sockfd, (char *)buffer, buffersize,
					MSG_WAITALL, ( struct sockaddr *) &cliaddr,
					&len);*/
		n = recvfrom(sockfd, (char *)buffer, buffersize,
					MSG_DONTWAIT, ( struct sockaddr *) &cliaddr,
					&len);

		output.clear(); // we clear it each time we try to receive. 
		//now I need to find if there is the word BYE in it
		if (n >= 0)
			ROS_DEBUG_STREAM_NAMED(debugger_sink, "Received data ok."<<n);
		else
		{
			//TODO: Change to warning throttle or something
			ROS_DEBUG_STREAM_NAMED(debugger_sink, "Received no data. From socket... Error No.: "<<n);
			return false;
		}
		buffer[n] = '\0';
			
		ROS_DEBUG_NAMED(debugger_sink, "Client : %s\n", buffer);
		if (strcmp(buffer, "BYE!") == 0 )
		{
		       ROS_INFO_STREAM_NAMED(debugger_sink, "Received goodbye SS OK.");
		       return false;
		}

		std::stringstream ss;
		ss << buffer;
		std::istream_iterator<std::string> begin(ss), end;
		std::vector<std::string> vstrings(begin, end);
		//std::copy(vstrings.begin(), vstrings.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

		//std::deque output;
		for (auto a_str : vstrings)
		{
			output.push_back(std::stod(a_str));
		}
		////THIS IS KINDA WORKING. I THINK
		//std::stringstream out_ros;
		//for (auto a_double : myvec )
		//	out_ros << a_double << "|";
		//ROS_DEBUG_STREAM("What I got: " << out_ros ) ;

		//printf("Client : %s\n", buffer);
		sendto(sockfd, (const char *)hello, strlen(hello),
			MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
				len);
		//todo: rosdebug or somehting
		ROS_DEBUG_NAMED(debugger_sink, "Hello message sent.\n");
		
		//return output;
		for( auto i:output)
			ROS_DEBUG_STREAM_NAMED(debugger_sink, "each num: " << i);
		//ROS_DEBUG_STREAM("THIS THING:" << output.size());
		return true;
	

}

