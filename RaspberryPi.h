#ifndef RASPBERRYPI_H
#define RASPBERRYPI_H

#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <cstring>
#include <iostream>
#include <cstdlib>
#include "sockLib.h"
#include "DriverStationLCD.h"

using namespace std;

class RaspberryPi
{
private:
	//Private variables
	int xPos, yPos;
	int sock;
	
public:
	//Public functions
	RaspberryPi(string port);
	~RaspberryPi();
	int GetXPos();
	int GetYPos();
	void Read();
};

#endif
