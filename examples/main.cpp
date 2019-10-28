#include <stdio.h>
#include <vector>

#include "../include/uskinCanDriver.h"

/* void afonso_master(std::vector <struct _single_node_reading> * reading)
{
  printf("\n<----------------------------------------------->\n");
  for (double i = 1; i <= 6;  i++) {
      for (double j = 4; j > 0; j--) {
        float norm = (((float)reading->back().z_data - 18300)/ (25500 - 18300))*100;
			printf("%.0f", norm);
			if ((int)norm/10 == 0) {
				printf("     ");
			}else if ((int)norm/100==0) {
				printf("    ");
			}else{
				printf("   ");
			}
      reading->pop_back();
		}
		printf("\n");
	}
} */

int main()
{

  UskinSensor uskin; // Will connect to "can0" device by default

  uskin_time_unit_reading *frame_reading = new uskin_time_unit_reading;
  frame_reading->instant_reading = new struct _uskin_node_time_unit_reading[uskin.GetUskinFrameSize()];

  // Used if we wish to filter incoming messages
  /*  struct can_filter rfilter[1];

  rfilter[0].can_id = 0x135;
  rfilter[0].can_mask = CAN_SFF_MASK; */
  //rfilter[1].can_id   = 0x101;
  //rfilter[1].can_mask = CAN_SFF_MASK;

  if (uskin.StartSensor())
  {
    uskin.GetFrameData_xyzValues(frame_reading);
    uskin.StopSensor();
  }

  for (int i = 0; i < frame_reading->number_of_nodes; i++)
  {
    std::cout << "Received message from device with CAN ID: " << std::hex << frame_reading->instant_reading[i].node_id << std::endl;
    std::cout << std::dec << "  With x value:  " << frame_reading->instant_reading[i].x_value << 
                 "  With y value:  " << frame_reading->instant_reading[i].y_value <<
                 "  With z value:  " << frame_reading->instant_reading[i].z_value << std::endl;
  }

  /*  while(true)
  {
      std::vector <struct _single_node_reading> instant_reading;
      driver.read_data(&instant_reading);

      afonso_master(&instant_reading);

  } */

  exit(0);
}
