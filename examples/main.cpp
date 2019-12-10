#include <stdio.h>
#include <vector>

#include "../include/uskinCanDriver.h"


// Check the results of this algorithm in outputed log file!!
int main()
{
  // Will connect to "can0" device by default.
  // By default 24 nodes are considered (6x4), but you can provide a different number (check constructors)
  UskinSensor *uskin = new UskinSensor; 
  _uskin_node_time_unit_reading *current_node_reading;

  // Start the uskin sensor
  if (!uskin->StartSensor())
  {
    printf("Problems initiating the sensor!");
    delete uskin;

    return (-1);
  }

  // Calibrate the sensor
  uskin->CalibrateSensor();

  // Save data to CSV file
  uskin->SaveData("uskin_data");
  uskin->SaveNormalizedData("uskin_normalized_data");

  uskin->RetrieveFrameData(); //Retrieves data and saves it in csv file

  for (int i = 0; i <= uskin->GetUskinFrameSize(); i++) // GetUskinFrameSize returns number of sensor nodes
  {
    if (current_node_reading = uskin->GetNodeData_xyzValues(i)) //Get magnet displacement values for each node 
      printf("%s\n", current_node_reading->to_str().c_str()); // Currently this is being printed to log file

  }

  uskin->NormalizeData(); // Normalized data with calibrated values and store it in csv file
  
  // Sleep and repeat the process
  sleep(2);

  uskin->RetrieveFrameData();

  for (int i = 0; i <= uskin->GetUskinFrameSize(); i++)
  {
    if (current_node_reading = uskin->GetNodeData_xyzValues(i))
      printf("%s", current_node_reading->to_str().c_str());
    
  }

  uskin->NormalizeData();

  uskin->StopSensor();

  delete uskin;

  exit(0);
}
