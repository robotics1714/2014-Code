#include "RaspberryPi.h"

RaspberryPi::RaspberryPi(string port)
{
	missingPacketCount = 0;
	
	int status;
	struct addrinfo hints;
	struct addrinfo* roboInfo;
	
	//Clear the hints
	memset(&hints, 0, sizeof(hints));
	
	//Get the address info for the robot
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE;
	status = getaddrinfo(NULL, port.c_str(), &hints, &roboInfo);
	if(status != 0)
	{
		cout<<"Error retrieving the address information \n";
	}
	
	//Create the socket
	sock = socket(roboInfo->ai_family, roboInfo->ai_socktype, roboInfo->ai_protocol);
	if(sock == -1)
	{
		cout<<"Error creating the socket\n";
	}
	
	//bind the socket to a port
	status = bind(sock, roboInfo->ai_addr, roboInfo->ai_addrlen);
	if(status != 0)
	{
		cout<<"Error binding the socket\n";
	}
	
	//Set the x and y position values to their error value (-2)
	xPos = -2;
	yPos = -2;
}

RaspberryPi::~RaspberryPi()
{
	close(sock);
}

int RaspberryPi::GetXPos()
{
	return xPos;
}

int RaspberryPi::GetYPos()
{
	return yPos;
}

void RaspberryPi::Read()
{
	char msg[5];//4 chars instead of 5 for the null termination
	memset(msg, 0, sizeof(msg));
	
	//Recieve a packet, if the function returns 0 no message was recieved
	if(recvfrom(sock, msg, sizeof(msg), MSG_DONTWAIT, NULL, NULL) == -1)
	{
		//printf("No data was recieved\n");
		//If the robot doesn't recieve so many packets in a row, assume we lost connection
		missingPacketCount++;
		if(missingPacketCount >= 500)
		{
			xPos = -2;
			yPos = -2;
		}
	}
	else
	{
		//Convert the string to the two position integers
		char xStr[2];
		char yStr[2];
		
		xStr[0] = msg[0];
		xStr[1] = msg[1];
		yStr[0] = msg[2];
		yStr[1] = msg[3];
		
		xPos = atoi(xStr);
		yPos = atoi(yStr);
		printf("It worked\n");
		
		//Reset the missing packet count
		missingPacketCount = 0;
	}
}
