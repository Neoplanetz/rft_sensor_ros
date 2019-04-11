#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <bitset>
#include <sys/time.h>

#include "RFT_IF_PACKET_Rev1.0.h"
#include "serial_port.h"

using namespace std;

bool robotous_read_model_name(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_serial_number(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_firmware_version(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_baud_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_filter_setting(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_output_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_force_once(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_force(SerialPort USBport, CRT_RFT_IF_PACKET& packet, bool save);
void robotous_read_current_sensor_settings(SerialPort USBport, CRT_RFT_IF_PACKET& packet);

bool robotous_set_baud_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char buad_rate);
bool robotous_set_filter(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char filter_type, unsigned char cutoff_frequency);
void robotous_set_stop_force_out(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_set_output_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char output_rate);
void robotous_set_bias(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char bias_is_on);


