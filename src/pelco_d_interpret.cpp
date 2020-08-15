// This is the code that will interpret the PELCO-D from the
// Security camera keyboard, which will eventually control the cameras.

#ifndef HEADER_H
#define HEADER_H
#include "headers.h"
#endif
#include "pelco_d_interface/cam_ctrl.h"

void interpretData(pelco_d_interface::cam_ctrl srv, ros::ServiceClient client);

int main (int argc, char **argv) {

// ROS initialisation
ros::init(argc, argv, "pelco_d_read");
ros::NodeHandle rosNode;
ros::ServiceClient client = rosNode.serviceClient<pelco_d_interface::cam_ctrl>("cam_ctrl");
pelco_d_interface::cam_ctrl camCtrlService;



// Get data stream from UART and interpret it.
  int fd;
  int count;
  unsigned int nextTime;

  if ((fd = serialOpen("/dev/ttyS0", 9600)) < 0)  {
    ROS_ERROR("Unable to open serial device; %s\n", strerror (errno));
    return 1;
  }

  bool readingValidData = false;
  unsigned char dataByteCounter = 0;
  while (rosNode.ok()) {
    while (serialDataAvail(fd))  {
      data = serialGetchar(fd);
      ROS_DEBUG("-> %d\n", data);
      fflush(stdout);
      // If byte is first byte (= to PD_SYNC) or if already reading the data. This
      // prevents unwanted data from being used.
      if(data == PD_SYNC || readingValidData) {
        readingValidData = true;
        recievedBytes[dataByteCounter] = data;
        dataByteCounter++; // Increment data byte number

        // Done reading all 7 bytes of data; interpret it.
        if(dataByteCounter == 7) {
          dataByteCounter = 0; // Reset data byte number
          readingValidData = false; // Data reading is done
          int sum = 0;
          for (unsigned char i = 1; i <= 5; i++)
            sum += recievedBytes[i];
          sum %= 256;
          ROS_DEBUG("Checksum: %d", sum);
          // Data is valid, interpret it.
          if(sum == READ_CHECKSUM)
            interpretData(camCtrlService, client);
          else ROS_ERROR("Checksums do not match.");
        }
      }
    }
  }
}

// Interprets data written to recievedBytes.
void interpretData(pelco_d_interface::cam_ctrl srv, ros::ServiceClient client) {
  static bool senseBit;
  senseBit = READ_SENSE;
  if(READ_ON_OFF){
      ROS_DEBUG("Turn camera %s. Toggling camera on/off.", senseBit?"on":"off");
      srv.request.cmd = srv.request.ON_OFF_TOGGLE;
  }
  else if(READ_IRIS_CLOSE) {
    ROS_DEBUG("Close camera iris.");
    srv.request.cmd = srv.request.IRIS_CLOSE;
  }
  else if(READ_IRIS_OPEN) {
    ROS_DEBUG("Open camera iris.");
    srv.request.cmd = srv.request.IRIS_OPEN;
  }
  else if(READ_FOCUS_NEAR) {
    ROS_DEBUG("Focus camera near.");
    srv.request.cmd = srv.request.FOCUS_NEAR;
  }
  else if(READ_FOCUS_FAR) {
    ROS_DEBUG("Focus camera far.");
    srv.request.cmd = srv.request.FOCUS_FAR;
  }
  else if(READ_ZOOM_WIDE) {
    ROS_DEBUG("Zoom out.");
    srv.request.cmd = srv.request.ZOOM_WIDE;
  }
  else if(READ_ZOOM_TELE) {
    ROS_DEBUG("Zoom in.");
    srv.request.cmd = srv.request.ZOOM_TELE;
  }
  else if(READ_DOWN || READ_UP) {
    srv.request.cmd = srv.request.PAN_TILT;
    srv.request.pan = READ_PAN_SPEED * (READ_DOWN?(-1):(1)); // Negate if going down
    ROS_DEBUG("Pan by %d", srv.request.pan);
  }
  else if(READ_LEFT || READ_RIGHT) {
    srv.request.cmd = srv.request.PAN_TILT;
    srv.request.tilt = READ_TILT_SPEED * (READ_LEFT?(-1):(1)); // Negate if going left
    ROS_DEBUG("Tilt by %d", srv.request.tilt);
  }
  else {
    ROS_DEBUG("Unknown or no srv.request.cmd; stopping all.");
    srv.request.cmd = srv.request.STOP_ALL;
  }

  srv.request.camera = READ_CAMERA_NUMBER;
  // Send info and get response
  if(client.call(srv)){
    ROS_DEBUG("Service called. Response = %d.", srv.response.error);
  }
  else ROS_ERROR("Failed to call service cam_ctrl");
}
