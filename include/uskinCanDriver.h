#ifndef USKINCANDRIVER_H
#define USKINCANDRIVER_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <array>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include "can_communication.h" // Our library for can communication

//#define DEBUG 1

#define USKIN_ROWS 4
#define USKIN_COLUMNS 6

#define ZNODEMAXREAD 25500
#define ZNODEMINREAD 18300

unsigned int convert_16bit_hex_to_dec(__u8 *data);

unsigned long convert_32bit_hex_to_dec(__u32 *data);

unsigned int convert_dec_to_16bit_hex_(__u8 *data);

unsigned long convert_dec_to_32bit_hex(__u32 *data);

struct _uskin_node_time_unit_reading
{
  __u32 node_id;
  int x_value;
  int y_value;
  int z_value;

  void clear()
  {
    node_id = 0x00000000;
    x_value = 0;
    y_value = 0;
    z_value = 0;
  }

  void normalize()
  {
    //x_value = (((float)x_value - NODEMINREAD) / (NODEMAXREAD - NODEMINREAD)) * 100;
    //y_value = (((float)y_value - NODEMINREAD) / (NODEMAXREAD - NODEMINREAD)) * 100;
    z_value = (((float)z_value - ZNODEMINREAD) / (ZNODEMAXREAD - ZNODEMINREAD)) * 100;
  }
};

struct uskin_time_unit_reading
{
  time_t timestamp;
  struct _uskin_node_time_unit_reading *instant_reading;
  int number_of_nodes;

  void clear()
  {
    for (int i = 0; i < number_of_nodes; i++)
    {
      instant_reading[i].clear();
    }

    number_of_nodes = 0;
  }

  void normalize()
  {
    for (int i = 0; i < number_of_nodes; i++)
    {
      instant_reading[i].normalize();
    }
  }
};

class UskinSensor
{
  // Access specifier

private:
  const int frame_size;

  const int frame_columns;

  const int frame_rows;

  std::string log_file = "uSkinCanDriver_log.output";

  CanDriver *driver;

  int sensor_has_started = 0;

  int data_is_being_saved = 0;

  int is_sensor_calibrated = 0;

  std::ofstream myfile;

  uskin_time_unit_reading *frame_reading;

public:
  UskinSensor();
  UskinSensor(int column_nodes, int row_nodes);
  ~UskinSensor();

  int StartSensor();
  int StopSensor();
  int GetUskinFrameSize();

  void CalibrateSensor(); // Leaving the sensor untouched for a period of time

  void GetFrameData_xyzValues();
  uskin_time_unit_reading *GetFrameData_xValues();
  uskin_time_unit_reading *GetFrameData_yValues();
  uskin_time_unit_reading *GetFrameData_zValues();

  uskin_time_unit_reading *GetNodeData_xyzValues(int node); // TODO convert node to 16bit hex
  uskin_time_unit_reading *GetNodeData_xValues(int node);   // TODO convert node to 16bit hex
  uskin_time_unit_reading *GetNodeData_yValues(int node);   // TODO convert node to 16bit hex
  uskin_time_unit_reading *GetNodeData_zValues(int node);   // TODO convert node to 16bit hex

  void PrintData();
  void SaveData();
  void SaveData(std::string filename);
};

#endif
