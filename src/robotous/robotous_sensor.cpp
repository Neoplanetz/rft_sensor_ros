#include "robotous_sensor.h"

///////////////// kbhit ///////////////////
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}
////////////////////////////////////////////

//using namespace std;


// ROBOTOUS F/T Sensor Library: Read model name
bool robotous_read_model_name(SerialPort USBport, CRT_RFT_IF_PACKET& packet) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// Fill packet packet_buff
	packet.UPG_read_product_name(command_packet_buff);
	
	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Receive response packet over USB
	USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

	// Read response packet
	return packet.rcvd_data_field_processing(response_packet_buff+1, CMD_GET_PRODUCT_NAME);
}

// ROBOTOUS F/T Sensor Library: Read serial number
bool robotous_read_serial_number(SerialPort USBport, CRT_RFT_IF_PACKET& packet) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// Fill packet packet_buff
	packet.UPG_read_serial_name(command_packet_buff);
	
	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Receive response packet over USB
	USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

	// Read response packet
	return packet.rcvd_data_field_processing(response_packet_buff+1, CMD_GET_SERIAL_NUMBER);
}

// ROBOTOUS F/T Sensor Library: Read Firmware version
bool robotous_read_firmware_version(SerialPort USBport, CRT_RFT_IF_PACKET& packet) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// Fill packet packet_buff
	packet.UPG_read_firmware_version(command_packet_buff);
	
	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Receive response packet over USB
	USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

	// Read response packet
	return packet.rcvd_data_field_processing(response_packet_buff+1, CMD_GET_FIRMWARE_VER);
}


// ROBOTOUS F/T Sensor Library: Read baud rate
bool robotous_read_baud_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// Fill packet packet_buff
	packet.UPG_read_comm_baudrate(command_packet_buff);
	
	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Receive response packet over USB
	USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

	// Read response packet
	return packet.rcvd_data_field_processing(response_packet_buff+1, CMD_GET_COMM_BAUDRATE);
}

// ROBOTOUS F/T Sensor Library: Read filter setting
bool robotous_read_filter_setting(SerialPort USBport, CRT_RFT_IF_PACKET& packet) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// Fill packet packet_buff
	packet.UPG_read_filter_type(command_packet_buff);
	
	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Receive response packet over USB
	USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

	// Read response packet
	return packet.rcvd_data_field_processing(response_packet_buff+1, CMD_GET_FT_FILTER);
}

// ROBOTOUS F/T Sensor Library: Read output rate
bool robotous_read_output_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// Fill packet packet_buff
	packet.UPG_read_output_frq(command_packet_buff);

	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Receive response packet over USB
	USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

	// Read response packet
	return packet.rcvd_data_field_processing(response_packet_buff+1, CMD_GET_CONT_OUT_FRQ);
}

// ROBOTOUS F/T Sensor Library: F/T 1 sample output
bool robotous_read_force_once(SerialPort USBport, CRT_RFT_IF_PACKET& packet) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// Fill packet packet_buff
	packet.UPG_read_force_once(command_packet_buff);

	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Receive response packet over USB
	USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

	// Read response packet
	return packet.rcvd_data_field_processing(response_packet_buff+1, CMD_FT_ONCE);
}

// ROBOTOUS F/T Sensor Library: F/T output
bool robotous_read_force(SerialPort USBport, CRT_RFT_IF_PACKET& packet, bool save) {

	bool ok = true;
	char char_in = 0;
	
	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// File descriptor
	ofstream myfile;

	// Open file if save is set
	if ( save ) {

		time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    char str_time[100], buffer[100];
		char *filename = "../output_file/FTdata-";

    strftime(str_time,80,"%Y-%m-%d-%H-%M-%S.txt",now);
		strcpy(buffer, filename);		
		strcat(buffer, str_time);

		myfile.open(buffer);

		myfile << "t [s]\tfx [N]\tfy [N]\tfz [N]\tmx [Nm]\tmy[Nm]\tmz [Nm]\r\n";
	}

	// Fill packet packet_buff
	packet.UPG_read_force(command_packet_buff);

	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Get initial time
	struct timeval start, end;
	long seconds, useconds;    
	float mtime;
  gettimeofday(&start, NULL);

	// Sensor reading
	while(true) {
		gettimeofday(&end, NULL);
		seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = (((seconds) * 1000 + useconds/1000.0) + 0.5) * 1E-3;

		if ( kbhit() ) {
			char_in = getchar();

			if ( char_in == 27 ) // Exit if the user presses ESC
				break;
			else if (char_in == 98) // Set bias if the user presses 'b'
				robotous_set_bias(USBport, packet, 1);
			else if (char_in == 99) // Set unbias if the user presses 'c'
				robotous_set_bias(USBport, packet, 0);
		}

		// Receive response packet over USB
		USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

		// Read response packet
		if ( packet.rcvd_data_field_processing(response_packet_buff+1, CMD_FT_CONT) ) {
			// Print values on the screen
			cout << "\033[1;37mRead Force/Torque sample:\033[0m\r\n\033[1;37mfx:\033[0m " << packet.m_rcvdForce[0] << "\r\n\033[1;37mfy:\033[0m " << packet.m_rcvdForce[1] << "\r\n\033[1;37mfz:\033[0m " << packet.m_rcvdForce[2] << "\r\n\033[1;37mmx:\033[0m " << packet.m_rcvdForce[3] << "\r\n\033[1;37mmy:\033[0m " << packet.m_rcvdForce[4] << "\r\n\033[1;37mmz:\033[0m " << packet.m_rcvdForce[5] << endl;

			// Overload byte
			bitset<6> overload(packet.m_rcvdForceStatus);
			cout << "\033[1;37mOverload status:\033[0m " << overload << endl;

			// Print values into the file
			myfile << mtime << "\t" << packet.m_rcvdForce[0] << "\t" << packet.m_rcvdForce[1] << "\t" << packet.m_rcvdForce[2] << "\t" << packet.m_rcvdForce[3] << "\t" << packet.m_rcvdForce[4] << "\t" << packet.m_rcvdForce[5] << "\r\n";
		}		
		else {
			ok = false;
			break;
		}
	}

	// Close file is save is set
	if ( save ) {
		myfile.close();
	}

	// Send force output stop
	robotous_set_stop_force_out(USBport, packet);

	return ok;
}

// ROBOTOUS F/T Sensor Library: Set baud rate
bool robotous_set_baud_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char baud_rate) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// Fill packet packet_buff
	packet.UPG_set_comm_baudrate(command_packet_buff, baud_rate);

	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Receive response packet over USB
	USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

	// Read response packet
	return packet.rcvd_data_field_processing(response_packet_buff+1, CMD_SET_COMM_BAUDRATE);
}

// ROBOTOUS F/T Sensor Library: Set filter
bool robotous_set_filter(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char filter_type, unsigned char cutoff_frequency) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// Fill packet packet_buff
	packet.UPG_set_filter_type(command_packet_buff, filter_type, cutoff_frequency);

	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Receive response packet over USB
	USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

	// Read response packet
	return packet.rcvd_data_field_processing(response_packet_buff+1, CMD_SET_FT_FILTER);
}

// ROBOTOUS F/T Sensor Library: Set Force Output Stop
void robotous_set_stop_force_out(SerialPort USBport, CRT_RFT_IF_PACKET& packet) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 

	// Fill packet packet_buff
	packet.UPG_set_stop_force_out(command_packet_buff);

	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);
}

// ROBOTOUS F/T Sensor Library: Set Output Rate
bool robotous_set_output_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char output_rate) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 
	unsigned char response_packet_buff[UART_RESPONSE_PACKET_SIZE];

	// Fill packet packet_buff
	packet.UPG_set_output_frq(command_packet_buff, output_rate);

	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);

	// Receive response packet over USB
	USBport.readSerial(response_packet_buff, UART_RESPONSE_PACKET_SIZE);

	// Read response packet
	return packet.rcvd_data_field_processing(response_packet_buff+1, CMD_SET_CONT_OUT_FRQ);
}

// ROBOTOUS F/T Sensor Library: Set Bias
void robotous_set_bias(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char bias_is_on) {

	unsigned char command_packet_buff[UART_COMMAND_PACKET_SIZE]; 

	// Fill packet packet_buff
	packet.UPG_set_bias(command_packet_buff, bias_is_on);

	// Send packet over USB
	USBport.writeSerial(command_packet_buff, UART_COMMAND_PACKET_SIZE);
}

// ROBOTOUS F/T Sensor Library: Read current sensor settings
void robotous_read_current_sensor_settings(SerialPort USBport, CRT_RFT_IF_PACKET& packet) {
	
	cout << "\r\n\033[1;32mROBOTOUS F/T Sensor settings\033[0m" << endl;	

	// Get sensor model
	if( robotous_read_model_name(USBport, packet) ) 
		printf("F/T Sensor model: %s\r\n", packet.m_rcvd_product_name); 
	else
		printf("Error during reading sensor model!\r\n");

	// Get serial number
	if( robotous_read_serial_number(USBport, packet) )
		printf("F/T Sensor serial number: %s\r\n", packet.m_rcvd_serial_number); 
	else
		printf("Error during reading sensor serial number!\r\n");

	// Get Firmware version
	if( robotous_read_firmware_version(USBport, packet) )
		printf("F/T Sensor Firmware version: %s\r\n", packet.m_rcvd_firmware_version); 
	else
		printf("Error during reading sensor firmware version!\r\n");

	// Get communication baud rate
	if( robotous_read_baud_rate(USBport, packet) ) {

		int current_baud_rate = 0; // [bps]
		int setting_baud_rate = 0; // [bps]
		switch (packet.m_rcvd_curr_comm_baudrate) {
			case UART_BAUDRATE_115200_DEFAULT:
				current_baud_rate = 115200;
				break;
			case UART_BAUDRATE_921600:
				current_baud_rate = 921600;
				break;
			case UART_BAUDRATE_460800:
				current_baud_rate = 460800;
				break;
			case UART_BAUDRATE_230400: 
				current_baud_rate = 230400;
				break;
			case UART_BAUDRATE_115200:
				current_baud_rate = 115200;
				break;
			case UART_BAUDRATE_57600:
				current_baud_rate = 57600;
				break;
		}

		switch (packet.m_rcvd_set_comm_baudrate) {
			case UART_BAUDRATE_115200_DEFAULT:
				setting_baud_rate = 115200;
				break;
			case UART_BAUDRATE_921600:
				setting_baud_rate = 921600;
				break;
			case UART_BAUDRATE_460800:
				setting_baud_rate = 460800;
				break;
			case UART_BAUDRATE_230400: 
				setting_baud_rate = 230400;
				break;
			case UART_BAUDRATE_115200:
				setting_baud_rate = 115200;
				break;
			case UART_BAUDRATE_57600:
				setting_baud_rate = 57600;
				break;
		}

		printf("Current baud rate: %d bps\r\n", current_baud_rate); 
		printf("Setting baud rate: %d bps\r\n", setting_baud_rate);
	}
	else
		printf("Error during reading communication baud rate!\r\n");

	// Get filter setting
	if( robotous_read_filter_setting(USBport, packet) ) {

		if ( packet.m_rcvd_filter_type == 0 || (packet.m_rcvd_filter_type == 1 && packet.m_rcvd_filter_setting_value == 0) )
			printf("Filter: OFF\r\n"); 
		else {
			
			int read_cutoff_freq = 0; // [Hz]
			switch (packet.m_rcvd_filter_setting_value) {
				case CUTOFF_1Hz:
					read_cutoff_freq = 1;
					break;
				case CUTOFF_2Hz:
					read_cutoff_freq = 2;
					break;
				case CUTOFF_3Hz:
					read_cutoff_freq = 3;
					break;
				case CUTOFF_5Hz:
					read_cutoff_freq = 5;
					break;
				case CUTOFF_10Hz:
					read_cutoff_freq = 10;
					break;
				case CUTOFF_20Hz:
					read_cutoff_freq = 20;
					break;
				case CUTOFF_30Hz:
					read_cutoff_freq = 30;
					break;
				case CUTOFF_40Hz:
					read_cutoff_freq = 40;
					break;
				case CUTOFF_50Hz:
					read_cutoff_freq = 50;
					break;
				case CUTOFF_100Hz:
					read_cutoff_freq = 100;
					break;
				case CUTOFF_150Hz:
					read_cutoff_freq = 150;
					break;
				case CUTOFF_200Hz:
					read_cutoff_freq = 200;
					break;
				case CUTOFF_300Hz:
					read_cutoff_freq = 300;
					break;
				case CUTOFF_500Hz:
					read_cutoff_freq = 500;
					break;
			}
			printf("Filter: ON\r\nCutoff frequency: %d Hz\r\n", read_cutoff_freq);
		}
	}
	else
		printf("Error during reading filter setting!\r\n");

	// Get output rate
	if( robotous_read_output_rate(USBport, packet) ) {

		int rate = 0; // [Hz]
		switch (packet.m_rcvd_tx_frq) {
			case OUTPUT_FRQ_200Hz_DEFAULT:
				rate = 200;
				break;
			case OUTPUT_FRQ_10Hz:
				rate = 10;
				break;
			case OUTPUT_FRQ_20Hz:
				rate = 20;
				break;
			case OUTPUT_FRQ_50Hz: 
				rate = 50;
				break;
			case OUTPUT_FRQ_100Hz:
				rate = 100;
				break;
			case OUTPUT_FRQ_200Hz:
				rate = 200;
				break;
			case OUTPUT_FRQ_333Hz:
				rate = 333;
				break;
			case OUTPUT_FRQ_500Hz:
				rate = 500;
				break;
			case OUTPUT_FRQ_1000Hz:
				rate = 1000;
				break;
		}
		printf("Output rate: %d Hz\r\n", rate); 
	}
	else
		printf("Error during reading output rate!\r\n");

}

