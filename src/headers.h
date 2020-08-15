#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "ros/ros.h"
#include "wiringSerial.h"
#include <wiringPi.h>

#include "serial_comm.cpp"

#define delay(a) usleep(a*1000)

unsigned char recievedBytes[7] = {0};
unsigned char data = 0;

// ============= Pelco-D Bytes =============
// Constant definitions
#define PD_SYNC 0xFF
// Command byte 1
#define READ_SENSE (((int)recievedBytes[2] & 0x80) != 0) // Sense bit dictates meaning of some other command bits
#define READ_SCAN (((int)recievedBytes[2] & 0x10) != 0) // Not on dome controller
#define READ_ON_OFF (((int)recievedBytes[2] & 0x08)) // Not on dome controller
#define READ_IRIS_CLOSE (((int)recievedBytes[2] & 0x04) != 0)
#define READ_IRIS_OPEN (((int)recievedBytes[2] & 0x02) != 0)
#define READ_FOCUS_NEAR (((int)recievedBytes[2] & 0x01) != 0)
// Command byte 2
#define READ_FOCUS_FAR (((int)recievedBytes[3] & 0x80) != 0)
#define READ_ZOOM_WIDE (((int)recievedBytes[3] & 0x40) != 0)
#define READ_ZOOM_TELE (((int)recievedBytes[3] & 0x20) != 0)
#define READ_DOWN (((int)recievedBytes[3] & 0x10) != 0)
#define READ_UP (((int)recievedBytes[3] & 0x08) != 0)
#define READ_LEFT (((int)recievedBytes[3] & 0x04) != 0)
#define READ_RIGHT (((int)recievedBytes[3] & 0x02) != 0)
// Other info in the data
#define READ_CAMERA_NUMBER recievedBytes[1]
#define READ_TILT_SPEED recievedBytes[4]
#define READ_PAN_SPEED recievedBytes[5]
#define READ_CHECKSUM recievedBytes[6]
