#include "serial_port.h"

using namespace std;

SerialPort::SerialPort() { }

SerialPort::~SerialPort() { }

SerialPort::SerialPort(char* port, speed_t baud) {
	COMport = port;
	baudrate = baud;
}

void SerialPort::setCOM(const char* port) {
	COMport = port;
}

void SerialPort::setBAUD(speed_t baud) {
	baudrate = baud;
}

int SerialPort::connect() {
	int fd;
	struct termios options;

	fd = open(COMport, O_RDWR| O_NOCTTY | O_NDELAY);

	if (fd == -1) {
		cout << "\033[1;31mError: Serial connnection failed!\033[0m\r\n";

		return -1;
	}
	else {
		fcntl(fd, F_SETFL, 0);
		tcgetattr(fd, &options);

		/* Set Baud Rate */
		cfsetospeed (&options, baudrate);
		cfsetispeed (&options, baudrate);

		/* Setting other Port Stuff */
		options.c_cflag     &=  ~PARENB;        // Make 8n1
		options.c_cflag     &=  ~CSTOPB;
		options.c_cflag     &=  ~CSIZE;
		options.c_cflag     |=  CS8;
		options.c_cflag     &=  ~CRTSCTS;       // no flow control
		options.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
		options.c_oflag     =   0;                  // no remapping, no delays
		options.c_cc[VMIN]      =   1;                  // read wait 1 byte
		options.c_cc[VTIME]     =   0;                  // 0 seconds read timeout

		options.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
		options.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
		options.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
		options.c_oflag     &=  ~OPOST;              // make raw

		/* Flush Port, then applies attributes */
		tcflush( fd, TCIFLUSH );

		if ( tcsetattr ( fd, TCSANOW, &options ) != 0) {
			cout << "Error " << errno << " from tcsetattr" << endl;
		}

		portDescr = fd;
		cout << "\033[1;31mSerial connection estabilished.\033[0m\r\n";

		return 0;
	}
}

int SerialPort::disconnect() {
	int err;
	err = close(portDescr);
	if(err == 0) {
		cout << "\033[1;31mSerial connection closed.\033[0m\r\n";
	}
	else {
		cout << "\033[1;31mError: cannot close serial communication!\033[0m\r\n";
	}

	return err;
}

int SerialPort::readSerial(unsigned char* buf, int num_byte) {

	int count = 0;

	for (int i = 0; i < num_byte; i++) {
		if( read(portDescr, buf+i, 1) > 0) {
			count++;
		}
	}
	return count;
}

int SerialPort::writeSerial(unsigned char* buf, int num_byte) {

	return write(portDescr, buf, num_byte);
}
