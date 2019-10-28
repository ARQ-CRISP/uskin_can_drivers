#include <string>
#include <iostream>
#include "uskinCanDriver.h"

unsigned int convert_16bit_hex_to_dec(__u8 *data)
{
  // We will always be converting Hex MSB and LSB (2 bytes) to Dec
  return long(data[0] << 8 | data[1]);
}

unsigned long convert_32bit_hex_to_dec(__u32 *data)
{
  // We will always be converting Hex MSB and LSB (2 bytes) to Dec
  return long(data[0] << 16 | data[1] << 8 | data[2]);
}

void storeNodeReading(struct _uskin_node_time_unit_reading * node_reading, struct can_frame *raw_node_reading)
{
  node_reading->node_id = raw_node_reading->can_id;
  node_reading->x_value = convert_16bit_hex_to_dec(&raw_node_reading->data[1]);
  node_reading->y_value = convert_16bit_hex_to_dec(&raw_node_reading->data[3]);
  node_reading->z_value = convert_16bit_hex_to_dec(&raw_node_reading->data[5]);

  /* std::cout << "Received message from device with CAN ID: " << node_reading.node_id << 
        "  With x value:  " << node_reading.x_value << 
        "  With y value:  " << node_reading.y_value <<
        "  With z value:  " << node_reading.z_value << std::endl;
 */
  delete raw_node_reading;
}

UskinSensor::UskinSensor() : frame_columns(USKIN_COLUMNS), frame_rows(USKIN_ROWS), frame_size(USKIN_COLUMNS * USKIN_ROWS)
{
  if (DEBUG)
    freopen((char *)log_file.c_str(), "w", stdout);

  driver = new CanDriver;

  return;
};

UskinSensor::UskinSensor(int column_nodes, int row_nodes) : frame_columns(column_nodes), frame_rows(row_nodes), frame_size(column_nodes * row_nodes)
{
  if (DEBUG)
    freopen((char *)log_file.c_str(), "w", stdout);

  driver = new CanDriver;

  return;
};
UskinSensor::~UskinSensor(){};

int UskinSensor::StartSensor()
{
  logInfo(1, ">> UskinSensor::StartSensor()");
  int return_value = 1;

  if (driver->openConnection() && driver->requestData())
  {
    logInfo(2, "The sensor has started successfully!");

    sensor_has_started = 1;
  }
  else
  {
    logError(2, "Problems starting the sensor");
    return_value = 0;
  }

  logInfo(1, "<< UskinSensor::StartSensor()");

  return return_value;
}
int UskinSensor::StopSensor()
{
  logInfo(1, ">> UskinSensor::StopSensor()");

  driver->stopData();
  sensor_has_started = 0;

  logInfo(2, "The sensor has been stoped");

  logInfo(1, "<< UskinSensor::StopSensor()");

  return 1;
};

int UskinSensor::GetUskinFrameSize()
{
  logInfo(1, ">> UskinSensor::GetUskinFrameSize()");
  logInfo(1, "<< UskinSensor::GetUskinFrameSize()");
  return frame_size;
};

int UskinSensor::NormalizeData(){}; // Leaving the sensor untouched for a period of time

void UskinSensor::GetFrameData_xyzValues(uskin_time_unit_reading * frame_reading)
{
  logInfo(1, ">> UskinSensor::GetFrameData_xyzValues()");

  struct can_frame *raw_data[frame_size];
  time_t timer;

  //std::shared_ptr<struct can_frame **> raw_data = std::make_shared<struct can_frame **> ;

  if (!sensor_has_started)
  {
    logError(2, "You must start the sensor first!!");
    return;
  }

  driver->readData(raw_data, frame_size);

  for (int i = 0; i < frame_size; i++)
  {
    storeNodeReading(&frame_reading->instant_reading[i], raw_data[i]);
  }

  frame_reading->number_of_nodes = frame_size;
  frame_reading->timesptamp = time(&timer);

  //raw_data.reset();

  logInfo(1, "<< UskinSensor::GetFrameData_xyzValues()");

  return;
};

uskin_time_unit_reading *UskinSensor::GetFrameData_xValues(){};
uskin_time_unit_reading *UskinSensor::GetFrameData_yValues(){};
uskin_time_unit_reading *UskinSensor::GetFrameData_zValues(){};

uskin_time_unit_reading *UskinSensor::GetNodeData_xyzValues(int node){}; // TODO convert node to 16bit hex

uskin_time_unit_reading *UskinSensor::GetNodeData_xValues(int node){}; // TODO convert node to 16bit hex
uskin_time_unit_reading *UskinSensor::GetNodeData_yValues(int node){}; // TODO convert node to 16bit hex
uskin_time_unit_reading *UskinSensor::GetNodeData_zValues(int node){}; // TODO convert node to 16bit hex
