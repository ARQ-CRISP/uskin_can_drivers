#ifndef USKINCANDRIVER_H
#define USKINCANDRIVER_H

#include <iostream>
#include <cstdio>
#include <array>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <memory>

#include <vector>

#include "can_communication.h" // Our library for can communication

//#define DEBUG 1

#define USKIN_ROWS 4
#define USKIN_COLUMNS 6

unsigned int convert_16bit_hex_to_dec(__u8 *data);

unsigned long convert_32bit_hex_to_dec(__u32 *data);

unsigned int convert_dec_to_16bit_hex_(__u8 *data);

unsigned long convert_dec_to_32bit_hex(__u32 *data);

struct uskin_time_unit_reading
{
  time_t timesptamp;
  struct _uskin_node_time_unit_reading *instant_reading;
  int number_of_nodes;
};

struct _uskin_node_time_unit_reading
{
  __u32 node_id;
  int x_value;
  int y_value;
  int z_value;
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

public:
  UskinSensor();
  UskinSensor(int column_nodes, int row_nodes);
  ~UskinSensor();

  int StartSensor();
  int StopSensor();
  int GetUskinFrameSize();
  int NormalizeData(); // Leaving the sensor untouched for a period of time

  void GetFrameData_xyzValues(uskin_time_unit_reading * frame_reading);
  uskin_time_unit_reading *GetFrameData_xValues();
  uskin_time_unit_reading *GetFrameData_yValues();
  uskin_time_unit_reading *GetFrameData_zValues();

  uskin_time_unit_reading *GetNodeData_xyzValues(int node); // TODO convert node to 16bit hex
  uskin_time_unit_reading *GetNodeData_xValues(int node);   // TODO convert node to 16bit hex
  uskin_time_unit_reading *GetNodeData_yValues(int node);   // TODO convert node to 16bit hex
  uskin_time_unit_reading *GetNodeData_zValues(int node);   // TODO convert node to 16bit hex
};

#endif
