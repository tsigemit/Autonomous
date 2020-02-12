/**
	Purpose: Controlling Wifibot-robot through RS232-serial communication. 
	@author: Burkin Gunke (burkin@student.chalmers.se)
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <termios.h> //header contains the definitions used by the terminal I/O interfaces
#include <unistd.h> //read() write() close()
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <sys/stat.h>
#include <stdint.h>
#include <math.h>
#define BAUDRATE B19200
#define SERIAL_PATH "/dev/ttyS0"

int serial_fd;
int serial_read_ret, serial_write_ret;
struct termios serial_settings;
char serial_buffer_send[9];
char serial_buffer_recv[22] = {0};
int Arr[7];

short Crc16(char *Adresse_tab , unsigned char Taille_max)
{
	unsigned int Crc = 0xFFFF;
	unsigned int Polynome = 0xA001;
	unsigned int CptOctet = 0;
	unsigned int CptBit = 0;
	unsigned int Parity= 0;
	Crc = 0xFFFF;
	Polynome = 0xA001;
	for ( CptOctet= 0 ; CptOctet < Taille_max ; CptOctet++)
	{
		Crc ^= *( Adresse_tab + CptOctet);
		for ( CptBit = 0; CptBit <= 7 ; CptBit++)
		{
			Parity= Crc;
			Crc >>= 1;
		if (Parity%2 == true) Crc ^= Polynome;
		}
	}
	return(Crc);
}

void cmdRCCallback(const geometry_msgs::Twist::ConstPtr& cmd_RC) //subscribe

{
	float m_straight=0;
	float m_turn=0;
	m_straight=abs(80*cmd_RC->linear.x);  							
	//linear.x max/min is +-1.5. 1.5*80 = 120, which is the max speed allowed for the robot's 		//first speed char.
	m_turn=abs((120/0.4)*cmd_RC->angular.z);
	// angular.z max/min is +-0.4. 

	serial_buffer_send[0]=(char)255;
	serial_buffer_send[1]=(char)0X07;
	serial_buffer_send[3]=(char)0X00;
	serial_buffer_send[5]=(char)0X00;

	if(cmd_RC->linear.x>0) // go forward
	{ 
		serial_buffer_send[2]=(char)m_straight; //speed1
		serial_buffer_send[4]=(char)m_straight; //speed2
		/*
		serial_buffer_send[3]=(char)m_straight; //speed1 extra
		serial_buffer_send[5]=(char)m_straight; //speed2 extra
		*/
		serial_buffer_send[6]=(char)81;
	}
	else if(cmd_RC->linear.x<0) // go backward
	{ 
		serial_buffer_send[2]=(char)m_straight;
		serial_buffer_send[4]=(char)m_straight;
		/*
		serial_buffer_send[3]=(char)m_straight; //extra
		serial_buffer_send[5]=(char)m_straight; //extra
		*/
		serial_buffer_send[6]=(char)1;
	}
	else if(cmd_RC->angular.z>0) // left turn
	{
		serial_buffer_send[2]=(char)m_turn;
		serial_buffer_send[4]=(char)m_turn;
		/*
		serial_buffer_send[3]=(char)m_turn; //extra
		serial_buffer_send[5]=(char)m_turn; //extra
		*/
		serial_buffer_send[6]=(char)17;
	}

	else if(cmd_RC->angular.z<0) // right turn
	{ 
		serial_buffer_send[2]=(char)m_turn;
		serial_buffer_send[4]=(char)m_turn;
		/*
		serial_buffer_send[3]=(char)m_turn; //extra
		serial_buffer_send[5]=(char)m_turn; //extra
		*/
		serial_buffer_send[6]=(char)65;
	}else
	{
		serial_buffer_send[2]=(char)0;
		serial_buffer_send[4]=(char)0;
		/*
		serial_buffer_send[3]=(char)0; //extra
		serial_buffer_send[5]=(char)0; //extra
		*/
		serial_buffer_send[6]=(char)1;
	}
	
	// printf("m_straight = %f \n", m_straight);
	//printf("m_turn = %f \n", m_turn);	
	short mycrcsend = Crc16(serial_buffer_send+1,6); // CRC encryption
	serial_buffer_send[7] = (char)mycrcsend;
	serial_buffer_send[8] = (char)(mycrcsend >> 8);
	serial_write_ret = write(serial_fd,serial_buffer_send,sizeof(serial_buffer_send));
}

int main(int argc, char **argv)
{
	printf("Opening %s in Read/Write mode at 19200 8-N-1... \n",SERIAL_PATH);
	fflush(stdout);
	//Try opening serial port
	serial_fd = open(SERIAL_PATH,O_RDWR|O_NOCTTY);
	if(serial_fd == -1) { //Checks the availability of the Serial Port
		printf("Failed.\n");
		fflush(stdout);
		return 0;
	} else {
		printf("Success.\n");
		fflush(stdout);
		tcgetattr(serial_fd, &serial_settings); //Get Current Settings of the Port
		cfsetispeed(&serial_settings,BAUDRATE); //Set Input Baudrate
		cfsetospeed(&serial_settings,BAUDRATE); //Set Output Baudrate
		serial_settings.c_cflag &= ~PARENB; //Mask Parity Bit as No Parity
		serial_settings.c_cflag &= ~CSTOPB; //Set Stop Bits as 1 or else it will be 2
		serial_settings.c_cflag &= ~CSIZE; //Clear the current no. of data bit setting
		serial_settings.c_cflag |= CS8; //Set no. of data bits as 8 Bits
	}
	ros::init(argc, argv, "LowLvCom");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 10, cmdRCCallback);

	ros::Rate r(200);
	if (ros::ok()){
		printf("Communicating with Wifibot! \n");
		ros::spin();
		//r.sleep();
	}
	return 0;
}
