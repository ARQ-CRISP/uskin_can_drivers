/*
 * Copyright: (C) 2019 CRISP, Advanced Robotics at Queen Mary,
 *                Queen Mary University of London, London, UK
 * Author: Rodrigo Neves Zenha <r.neveszenha@qmul.ac.uk>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */
/**
 * \file uskinCanDriver.h
 *
 * \author Rodrigo Neves Zenha
 * \copyright  Released under the terms of the GNU GPL v3.0.
 */

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

// Default for 4x6 uSkin version
#define USKIN_ROWS 4
#define USKIN_COLUMNS 6

// Hardcoded values retrieved from trial & error
#define XNODEMAXREAD 45000
#define YNODEMAXREAD 23000
#define ZNODEMAXREAD 25600

//#define ZNODEMINREAD 18300 // not used

static unsigned long int **frame_min_reads;
static int frame_min_reads_size;
// static unsigned long int **frame_max_reads;

//###################### Utils #########################

void open_log_file(std::string file_name);

//###################### Data Structures #########################
struct _uskin_node_time_unit_reading
{
  __u32 node_id;
  int sequence; // Index on frame's array of readings
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
    if (frame_min_reads_size > sequence) // Validating if frame_min_reads has already been initialized (from calibration)
    {
      std::stringstream converted_msg;

      converted_msg << "Normalizing node with CAN ID: " << std::hex << node_id << " With Sequence: " << std::dec << sequence << " Using the following minimum readings, X: " << frame_min_reads[sequence][0] << " Y: " << frame_min_reads[sequence][1] << " Z: " << frame_min_reads[sequence][2] << std::endl;
      logInfo(4, converted_msg.str());
      x_value = (((float)x_value - frame_min_reads[sequence][0]) / (XNODEMAXREAD - frame_min_reads[sequence][0])) * 100;
      y_value = (((float)y_value - frame_min_reads[sequence][1]) / (YNODEMAXREAD - frame_min_reads[sequence][1])) * 100;
      z_value = (((float)z_value - frame_min_reads[sequence][2]) / (ZNODEMAXREAD - frame_min_reads[sequence][2])) * 100;
      if (z_value < 0)
        z_value = 0; // Force Z to be above 0. Z would only get negative values if a node is being "pulled", which should not happen
    }
  }

  std::string to_str()
  {
    std::stringstream output;

    output << "node_id: " << std::hex << node_id << std::dec << " x_value:  " << x_value << " y_value:  " << y_value << " z_value:  " << z_value << std::endl;
    return output.str();
  }
};

struct uskin_time_unit_reading
{
  struct timeval timestamp;
  struct _uskin_node_time_unit_reading *instant_reading;
  int number_of_nodes = 0;

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
    logInfo(3, "Normalizing data with the following minimum and maximum readings:");
    logInfo(3, "Sizeof frame min reads:" + std::to_string(frame_min_reads_size));

    if (frame_min_reads_size == number_of_nodes) // Validating if frame_min_reads_size has already been initialized (from calibration)
    {
      logInfo(3, "Normalizing data with the following minimum and maximum readings:");
      for (int i = 0; i < number_of_nodes; i++)
      {
        instant_reading[i].normalize();
      }
    }
    else
      logError(3, "Problems normalizing sensor frame readings");
  }
};

void NormalizeData(uskin_time_unit_reading *frame);

void storeNodeReading(struct _uskin_node_time_unit_reading *node_reading, struct can_frame *raw_node_reading, int sequence);

//###################### UskinSensor #########################
class UskinSensor
{
  // Access specifier

private:
  // Sensor's frame_size is the total number of sensitive nodes in its frame (frame_columns*frame_rows)
  const int frame_size;

  const int frame_columns;

  const int frame_rows;

  // Log file
  std::string log_file = "uSkinCanDriver_log";
  // std::string log_file = "../log_files/uSkinCanDriver_log_";

  // Driver for CAN communication
  CanDriver *driver;

  // Flags if data has been properly started
  int sensor_has_started = 0;

  // Flags if sensor has been calibrated
  int sensor_is_calibrated = 0;

  // Flags if sensor is being stored in CSV file
  int data_is_being_saved = 0;

  int normalized_data_is_being_saved = 0;

  // CSV file for storing all data retrieved
  std::ofstream csv_file, normalized_csv_file;

  // Sensor's readings from all the sensitive nodes that compose it's frame
  uskin_time_unit_reading *frame_reading;

  void initializeCSVdataStructure(std::ofstream *csv);

public:
  UskinSensor();
  UskinSensor(std::string new_log_file);
  UskinSensor(int column_nodes, int row_nodes);
  UskinSensor(int column_nodes, int row_nodes, std::string new_log_file);
  ~UskinSensor();

  int StartSensor();
  int StopSensor();
  int GetUskinFrameSize();

  void CalibrateSensor(); // Leaving the sensor untouched for a period of time

  void RetrieveFrameData();

  _uskin_node_time_unit_reading *GetNodeData_xyzValues(int node);
  uskin_time_unit_reading *GetFrameData_xValues();
  uskin_time_unit_reading *GetFrameData_yValues();
  uskin_time_unit_reading *GetFrameData_zValues();

  // uskin_time_unit_reading *GetNodeData_xValues(int node); // TODO convert node to 16bit hex
  // uskin_time_unit_reading *GetNodeData_yValues(int node); // TODO convert node to 16bit hex
  // uskin_time_unit_reading *GetNodeData_zValues(int node); // TODO convert node to 16bit hex

  void PrintData();
  void SaveData();
  void SaveData(std::string filename);

  void SaveNormalizedData();
  void SaveNormalizedData(std::string filename);

  bool get_sensor_status();

  bool get_sensor_calibration_status();

  bool get_sensor_saved_data_status();

  void retrieveSensorMinReadings(int number_of_readings);
  void NormalizeData();
};

#endif
