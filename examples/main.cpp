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

  UskinSensor * uskin = new UskinSensor; // Will connect to "can0" device by default
  time_t timer;

  // Used if we wish to filter incoming messages
  /*  struct can_filter rfilter[1];

  rfilter[0].can_id = 0x135;
  rfilter[0].can_mask = CAN_SFF_MASK; */
  //rfilter[1].can_id   = 0x101;
  //rfilter[1].can_mask = CAN_SFF_MASK;
  time(&timer);
  uskin->SaveData("uskin_data_"+ std::to_string(timer) + ".csv"); //Only called once, will save output to a csv file


  if (uskin->StartSensor())
  {
    uskin->CalibrateSensor(); // Calibrates sensor and data is stored normalized
    uskin->GetFrameData_xyzValues(); // Get data. If SavaData was called, data is stored in file
    //uskin->PrintData();

    sleep(2);

    uskin->GetFrameData_xyzValues();


    uskin->StopSensor();
  }

  delete uskin;



  /*  while(true)
  {
      std::vector <struct _single_node_reading> instant_reading;
      driver.read_data(&instant_reading);

      afonso_master(&instant_reading);

  } */

  exit(0);
}
