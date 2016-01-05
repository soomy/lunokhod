/*
 * com_listener.cpp
 *
 *  Created on: Dec 25, 2015
 *      Author: manu
 *
 *  This reimplements the robocan com listener in c++
 */

#include <ros/ros.h>
#include <robocan_link/robocan_servo.h>
#include <robocan_link/robocan_io.h>
#include <robocan_link/robocan_adc.h>
#include <robocan_link/robocan_adc_time.h>
#include <robocan_link/robocan_getio.h>
#include <sstream>

// for sigint handler
#include <signal.h>

// for serial port operations
#include <fcntl.h>

// ---------------------------------------------
// globals

// send debug info to console?
bool quiet;

// serial port options
int serial_port;
static const char* port_name = "/dev/ttyUSB0";
static const int baud_rate = 115200;

// buffers and counters
int message_counter = 0;
std::string serial_buffer = "";

// publishers and subscribers
ros::Publisher pub_io;
ros::Publisher pub_adc;
ros::Subscriber sub_io;
ros::Subscriber sub_adc;
ros::Subscriber sub_servo;

// listener "thread"
ros::Timer listenTimer;

// topic names
std::string topic_pub_io =    "/cmd_can_io";
std::string topic_pub_adc =   "/cmd_can_adc";
std::string topic_sub_io =    "/cmd_can_getio";
std::string topic_sub_adc =   "/cmd_can_adc_time";
std::string topic_sub_servo = "/cmd_can_servo";

// --------------------------------------------------------------------
// utility functions

void sendSerialMessage(std::string data)
{
	if(-1 == serial_port)
	{
		ROS_ERROR("Cannot send message do serial port: port is not open.");
	}
	else
	{
		int success = write(serial_port, data.c_str(), data.length());

		if(-1 == success)
		{
			ROS_WARN("Failed to send Message to serial port: %s", data.c_str());
		}
	}
}

void readSerialBuffer()
{
	if(-1 == serial_port)
	{
		ROS_ERROR("Cannot read from serial port: port is not open.");
	}
	else
	{
		// read everything from the serial port in 64 byte chunks
		char data[64];
		while(0 != read(serial_port, data, 64))
		{
			serial_buffer += data;
		}
	}
}

void checkCanMsg(std::string message)
{
	try
	{
		int type = std::strtol(message.substr(0,3).c_str(), NULL, 16);
		int data[8];
		for(int i = 0; i < 8; i++)
		{
			data[i] = std::strtol(message.substr(4+2*i, 2).c_str(), NULL, 16);
		}

		if(0x180 < type && 0x200 > type)
		{
			if(!quiet) ROS_INFO("PDO1 Received (IO)");
			robocan_link::robocan_io msg_io;
			msg_io.printid = type - 0x180;
			msg_io.io0 = data[0] & 0b0001;
			msg_io.io1 = data[1] & 0b0010;
			msg_io.io2 = data[2] & 0b0100;
			msg_io.io3 = data[3] & 0b1000;
			pub_io.publish(msg_io);
		}
		else if(0x280 < type && 0x300 > type)
		{
			if(!quiet) ROS_INFO("PDO2 Received (ADC)");
			robocan_link::robocan_adc msg_adc;
			msg_adc.printid = type - 0x180;
			msg_adc.adc0 = data[0]*0xFF + data[1];
			msg_adc.adc1 = data[2]*0xFF + data[3];
			msg_adc.adc2 = data[4]*0xFF + data[5];
			msg_adc.adc3 = data[6]*0xFF + data[7];
			pub_adc.publish(msg_adc);
		}
		else if(0x380 < type && 0x400 > type)
		{
			if(!quiet) ROS_INFO("PDO3 Received (Servo)");
		}
		else
		{
			ROS_WARN("Received unrecognized message from serial port");
		}
	}
	catch (...)
	{
		ROS_ERROR("Unable to decode message from serial port");
	}
}

int openSerialPort()
{
	// open serial port
	int retries = 0;
	do
	{
		if(!quiet) ROS_INFO("Trying to open %s, try %d", port_name, retries + 1);
	    serial_port = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
		sleep(2);
	} while (3 > retries++ && 0 > serial_port);

	if(-1 == serial_port)
	{
    	ROS_ERROR("Unable to open %s", port_name);
	}
}

// SIGINT HANDLER
void mySigintHandler(int sig)
{
	listenTimer.Stop();
	
	// close serial port
	close(serial_port);

	sleep(1);
	ros::shutdown();
}

// --------------------------------------------------------------------
// callbacks

void cb_getio(const robocan_link::robocan_getio::ConstPtr& ptr_msg)
{
    if(!quiet) ROS_INFO("getio callback");
    std::stringstream s;
	s << "t" << 200+ptr_msg->printid << "0000000000000000\r";
	sendSerialMessage(s.str());
}

void cb_adc(const robocan_link::robocan_adc_time::ConstPtr& ptr_msg)
{
	if(!quiet) ROS_INFO("adc_time callback");
	std:: stringstream s;
	s << "t"  << 600+ptr_msg->printid << "8002005" << boost::format("%|04|")%(ptr_msg->repetition_time) << "000000\r";
	sendSerialMessage(s.str());
}

void cb_servo(const robocan_link::robocan_servo::ConstPtr& ptr_msg)
{
	if(!quiet) ROS_INFO("servo callback");
	std::stringstream s;
    s << "t" << boost::format("%|02|")%(400 + ptr_msg->servoid / 10) << "8"
      << boost::format("%|02|")%(ptr_msg->servoid % 10) << ptr_msg->value << "000000000000\r";
    sendSerialMessage(s.str());
}

void serialListen(const ros::TimerEvent& e)
{
	readSerialBuffer();

	int rPos = 0;
	// as long as there are "\r" characters in the buffer, we have messages to process
	while(serial_buffer.npos != (rPos = serial_buffer.find("\r")))
	{
		message_counter++;

		// check the first message, which is the buffer until the first occurrence of "\r"
		checkCanMsg(serial_buffer.substr(0, rPos));

		// crop the buffer to the second half (excluding the "\r" from the first message
		serial_buffer = serial_buffer.substr(rPos + 2);
	}
}

// --------------------------------------------------------------------
// main

int main(int argc, char** argv)
{
	// node initialization
	ros::init(argc, argv, "com_listener");
	ros::NodeHandle nh("~");

	nh.param("quiet", quiet, false);

	// publishers (forward messages from serial to ros)
	pub_io  = nh.advertise<robocan_link::robocan_io> (topic_pub_io,  10);
	pub_adc = nh.advertise<robocan_link::robocan_adc>(topic_pub_adc, 10);

	// subscribers (forward messages from ros to serial)
	sub_io =    nh.subscribe(topic_sub_io,    10, cb_getio);
	sub_adc =   nh.subscribe(topic_sub_adc,   10, cb_adc);
	sub_servo = nh.subscribe(topic_sub_servo, 10, cb_servo);

    // custom shutdown signal handler
	signal(SIGINT, mySigintHandler);

	// open serial port
	openSerialPort();

	// start serial port listening timer
    listenTimer = nh.createTimer(ros::Duration(0.01), serialListen);

	// ros loop
	ros::spin();

	return 0;
}
