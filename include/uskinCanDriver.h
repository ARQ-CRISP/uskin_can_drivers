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
#define YNODEMAXREAD 25000
#define ZNODEMAXREAD 25600

//#define ZNODEMINREAD 18300 // not used

static unsigned long int **frame_min_reads;
static int frame_min_reads_size = 0;
// static unsigned long int **frame_max_reads;

//###################### Utils #########################

void open_log_file(std::string file_name);

//###################### Data Structures #########################
struct _uskin_node_time_unit_reading
{
  __u32 node_id;
  int index;
  int x_value;
  int y_value;
  int z_value;
  int x_value_normalized;
  int y_value_normalized;
  int z_value_normalized;

  void clear()
  {
    node_id = 0x00000000;
    x_value = 0;
    y_value = 0;
    z_value = 0;
    x_value_normalized = 0;
    y_value_normalized = 0;
    z_value_normalized = 0;
  }

  void normalize()
  {
    if (frame_min_reads_size != 0) // Validating if frame_min_reads has already been initialized (from calibration)
    {
      std::stringstream converted_msg;

      converted_msg << "Normalizing node with CAN ID: " << std::hex << node_id << " With Index: " << std::dec << index << " Using the following minimum readings, X: " << frame_min_reads[index][0] << " Y: " << frame_min_reads[index][1] << " Z: " << frame_min_reads[index][2] << std::endl;
      logInfo(4, converted_msg.str());
      x_value_normalized = (((float)x_value - frame_min_reads[index][0]) / (XNODEMAXREAD - frame_min_reads[index][0])) * 100;
      y_value_normalized = (((float)y_value - frame_min_reads[index][1]) / (YNODEMAXREAD - frame_min_reads[index][1])) * 100;
      z_value_normalized = (((float)z_value - frame_min_reads[index][2]) / (ZNODEMAXREAD - frame_min_reads[index][2])) * 100;
      // if (z_value < 0)
      //   z_value = 0; // Force Z to be above 0. Z would only get negative values if a node is being "pulled", which should not happen
      // Force normalized valies to be within boundaries
      z_value_normalized < 0 ? z_value_normalized = 0 : (z_value_normalized > 100 ? z_value_normalized = 100 : z_value_normalized);
      y_value_normalized < -100 ? y_value_normalized = -100 : (y_value_normalized > 100 ? y_value_normalized = 100 : y_value_normalized);
      x_value_normalized < -100 ? x_value_normalized = -100 : (x_value_normalized > 100 ? x_value_normalized = 100 : x_value_normalized);
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

    if (frame_min_reads_size != 0) // Validating if frame_min_reads_size has already been initialized (from calibration)
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

void storeNodeReading(struct _uskin_node_time_unit_reading *node_reading, struct can_frame *raw_node_reading);

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

  int convertCanIDtoIndex(canid_t can_id);
  int convertIndextoCanID(int index);


  int StartSensor();
  int StopSensor();
  int GetUskinFrameSize();

  void CalibrateSensor(); // Leaving the sensor untouched for a period of time

  unsigned long int ** getCalibrationValues();

  void RetrieveFrameData();

  _uskin_node_time_unit_reading *GetNodeData_xyzValues(int node);
  _uskin_node_time_unit_reading *GetFrameData();

  uskin_time_unit_reading *GetFrameData_xValues();
  uskin_time_unit_reading *GetFrameData_yValues();
  uskin_time_unit_reading *GetFrameData_zValues();

  // uskin_time_unit_reading *GetNodeData_xValues(int node); // TODO convert node to 16bit hex
  // uskin_time_unit_reading *GetNodeData_yValues(int node); // TODO convert node to 16bit hex
  // uskin_time_unit_reading *GetNodeData_zValues(int node); // TODO convert node to 16bit hex

  void PrintData();
  void PrintNormalizedData();

  void SaveData();
  void SaveData(std::string filename);

  void SaveNormalizedData();
  void SaveNormalizedData(std::string filename);

  bool get_sensor_status();

  bool get_sensor_calibration_status();

  bool get_sensor_saved_data_status();

  void retrieveSensorMinReadings(int number_of_readings);
  bool NormalizeData();
};

#endif
