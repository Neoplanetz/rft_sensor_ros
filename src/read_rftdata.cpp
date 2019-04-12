#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"

#include "robotous_sensor.h"  //it includes also serial_port.h and RFT_IF_PACKET_Rev1.0.h

// Main of the node
int main(int argc, char **argv)
{
	// ROS initialization
	ros::init(argc, argv, "read_rftdata");
	  
	// NodeHandle 
	ros::NodeHandle n;
	
	// Publisher
	ros::Publisher readdata_pub = n.advertise<geometry_msgs::WrenchStamped>("rft_data", 1);
	
	// Setting of ROS rate [Hz]
	ros::Rate loop_rate(1000.0); 

	// Setup UART communication and sensor initialization
	SerialPort USB;
	char *COMport1 = "/dev/";
	char COMport[100];
	strcpy(COMport, COMport1);
	
	if (argc == 2) {
		strcat(COMport,argv[1]);
	}
	else {
		strcat(COMport,"ttyUSB0");
	}
	USB.setCOM(COMport);
	
	USB.setBAUD(B921600);
	//USB.setBAUD(B115200);
	//USB.setBAUD(B460800);


	if ( USB.connect() != 0 ) {
		cout << "\033[1;31m\n\nError setting USB connection!\033[1;31m" << endl;
		return -1;
	}
	
	// Sensor Packet definition
	CRT_RFT_IF_PACKET robotous_packet;
	
	 
	//Set BAUD RATE
	//robotous_set_baud_rate(USB, robotous_packet, UART_BAUDRATE_921600);
	//robotous_set_baud_rate(USB, robotous_packet, UART_BAUDRATE_460800);
	
	//Set output frequency
    //robotous_set_output_rate(USB, robotous_packet, OUTPUT_FRQ_1000Hz);
	//robotous_set_output_rate(USB, robotous_packet, OUTPUT_FRQ_500Hz);
	
	
	//Set LPF filter
	//robotous_set_filter(USB, robotous_packet, 1, CUTOFF_20Hz);
	robotous_set_filter(USB, robotous_packet, 1, CUTOFF_200Hz);
	//robotous_set_filter(USB, robotous_packet, 0, CUTOFF_20Hz);

        sleep(1);

	// Check communication with the sensor
	bool Ok = robotous_read_baud_rate(USB, robotous_packet);

	if (!Ok) {
		cout << "\n\n\033[1;31m    Communication with sensor failed!      \033[1;31m" << endl; 
	    USB.disconnect();  
		return -1;
	}
	
sleep(1);
	// Read current sensor settings
	robotous_read_current_sensor_settings(USB, robotous_packet);
 sleep(1); 
	// Sensor unbiasing
	robotous_set_bias(USB, robotous_packet, 1);
sleep(1);
	if( robotous_read_force_once(USB, robotous_packet) ) {
			cout << "\033[1;37mFirst received Force/Torque sample:\033[0m\r\n\033[1;37mfx:\033[0m " 
		   << robotous_packet.m_rcvdForce[0] << "\r\n\033[1;37mfy:\033[0m " 
		   << robotous_packet.m_rcvdForce[1] << "\r\n\033[1;37mfz:\033[0m " 
		   << robotous_packet.m_rcvdForce[2] << "\r\n\033[1;37mmx:\033[0m " 
		   << robotous_packet.m_rcvdForce[3] << "\r\n\033[1;37mmy:\033[0m " 
		   << robotous_packet.m_rcvdForce[4] << "\r\n\033[1;37mmz:\033[0m " 
		   << robotous_packet.m_rcvdForce[5] << endl;
	
			while (ros::ok()) {
				// Message object
				geometry_msgs::WrenchStamped ft_msg;

				// Read F/T data via USB port
				robotous_read_force_once(USB, robotous_packet);

				// Filling in the message content
				ft_msg.wrench.force.x = robotous_packet.m_rcvdForce[0];
				ft_msg.wrench.force.y = robotous_packet.m_rcvdForce[1];
				ft_msg.wrench.force.z = robotous_packet.m_rcvdForce[2];

				ft_msg.wrench.torque.x = robotous_packet.m_rcvdForce[3];
				ft_msg.wrench.torque.y = robotous_packet.m_rcvdForce[4];
				ft_msg.wrench.torque.z = robotous_packet.m_rcvdForce[5];
      	ft_msg.header.stamp = ros::Time::now();

                bitset<6> overload(robotous_packet.m_rcvdForceStatus);

				if (overload.any())
					cout << "\033[1;33mWarning: sensor overload\033[1;33m" << endl;

				// Publish the message
    	  readdata_pub.publish(ft_msg);

				// Needed only in case of a subscriber to run the callbacks!
				ros::spinOnce();

				loop_rate.sleep();
			}
  }

	else {
		cout << "\033[1;31mError during Force/Torque reading!\033[1;31m" << endl; 	
		USB.disconnect();
		return -1;
	}
	
	USB.disconnect();
	return 0;
}
