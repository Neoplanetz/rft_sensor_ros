#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

class SerialPort {
	public:
		const char* COMport;    // Number of COM port
		speed_t baudrate;  // Port baud rate
		int portDescr;     // Port descriptor

	public:
		SerialPort();
		SerialPort(char* port, speed_t baud);
		~SerialPort();

		void setCOM(const char* port);
		void setBAUD(speed_t baud);
		int connect();
		int disconnect();
		int readSerial(unsigned char* buf, int num_byte);
		int writeSerial(unsigned char* buf, int num_byte);
};
