/*
 * Copyright: (C) 2019 CRISP, Advanced Robotics at Queen Mary,
 *                Queen Mary University of London, London, UK
 * Author: Rodrigo Neves Zenha <r.neveszenha@qmul.ac.uk>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */
/**
 * \file uskinCanDriver.cpp
 *
 * \author Rodrigo Neves Zenha
 * \copyright  Released under the terms of the GNU GPL v3.0.
 */

#include <string>
#include <iostream>
#include "../include/uskinCanDriver.h"

//###################### Utils #########################

// Opening log file with timestamp
void open_log_file(std::string file_name)
{
  if (DEBUG)
  {
    time_t timer;
    struct tm *timeinfo;
    std::stringstream complete_file_name;
    char time_string[20];

    complete_file_name << file_name << "_";

    time(&timer);
    timeinfo = localtime(&timer);
    strftime(time_string, 20, "%F-%H-%M-%S", timeinfo);

    complete_file_name << time_string << ".output";

    freopen((char *)complete_file_name.str().c_str(), "w", stdout);
  }
}

//###################### Data Structures #########################

// Store can_frame data in _uskin_node_time_unit_reading structure
void storeNodeReading(struct _uskin_node_time_unit_reading *node_reading, struct can_frame *raw_node_reading, int index)
{
  node_reading->node_id = raw_node_reading->can_id;
  node_reading->index = index;
  node_reading->x_value = convert_16bit_hex_to_dec(&raw_node_reading->data[1]);
  node_reading->y_value = convert_16bit_hex_to_dec(&raw_node_reading->data[3]);
  node_reading->z_value = convert_16bit_hex_to_dec(&raw_node_reading->data[5]);

  delete raw_node_reading;
}

//###################### UskinSensor #########################

// Uskin constructors and destructor. Column and row numbers of nodes can be provided
UskinSensor::UskinSensor() : frame_columns(USKIN_COLUMNS), frame_rows(USKIN_ROWS), frame_size(USKIN_COLUMNS * USKIN_ROWS)
{
  open_log_file(log_file);

  driver = new CanDriver;

  frame_reading = new uskin_time_unit_reading;
  frame_reading->instant_reading = new struct _uskin_node_time_unit_reading[frame_size];
  frame_reading->number_of_nodes = frame_size;

  return;
};

UskinSensor::UskinSensor(std::string new_log_file) : frame_columns(USKIN_COLUMNS), frame_rows(USKIN_ROWS), frame_size(USKIN_COLUMNS * USKIN_ROWS)
{
  open_log_file(new_log_file);

  driver = new CanDriver;

  frame_reading = new uskin_time_unit_reading;
  frame_reading->instant_reading = new struct _uskin_node_time_unit_reading[frame_size];
  frame_reading->number_of_nodes = frame_size;

  return;
};

UskinSensor::UskinSensor(int column_nodes, int row_nodes) : frame_columns(column_nodes), frame_rows(row_nodes), frame_size(column_nodes * row_nodes)
{
  open_log_file(log_file);

  driver = new CanDriver;

  frame_reading = new uskin_time_unit_reading;
  frame_reading->instant_reading = new struct _uskin_node_time_unit_reading[frame_size];
  frame_reading->number_of_nodes = frame_size;

  return;
};

UskinSensor::UskinSensor(int column_nodes, int row_nodes, std::string new_log_file) : frame_columns(column_nodes), frame_rows(row_nodes), frame_size(column_nodes * row_nodes)
{
  open_log_file(new_log_file);

  driver = new CanDriver;

  frame_reading = new uskin_time_unit_reading;
  frame_reading->instant_reading = new struct _uskin_node_time_unit_reading[frame_size];
  frame_reading->number_of_nodes = frame_size;

  return;
};

UskinSensor::~UskinSensor()
{
  delete driver;
  delete frame_reading->instant_reading;
  delete frame_reading;

  if (get_sensor_calibration_status())
  {
    for (int i = 0; i < frame_size; i++)
    {
      delete frame_min_reads[i];
      // delete frame_max_reads[i];
    }
    delete frame_min_reads;
    // delete frame_max_reads;
  }

  if (data_is_being_saved)
    csv_file.close();
};

int UskinSensor::convertCanIDtoIndex(canid_t can_id)
{
  int index;

  // Convert canID to a frame_reading valid index
  index = (convert_dec_to_24bit_hex(can_id) % 10) * frame_rows + (convert_dec_to_24bit_hex(can_id) / 10 % 10);

  return index;
}

int UskinSensor::convertIndextoCanID(int index)
{
  canid_t can_id;

  // Convert a frame_reading valid index to canID
  can_id = (index % frame_rows) * 10 + (index / frame_rows) + 100;

  return can_id;
}

// Open connection and request data
int UskinSensor::StartSensor()
{
  logInfo(1, ">> UskinSensor::StartSensor()");
  int return_value = 1;

  if (driver->openConnection() && driver->requestData())
  {

    //CalibrateSensor();
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

// Stop data transmission
int UskinSensor::StopSensor()
{
  logInfo(1, ">> UskinSensor::StopSensor()");

  driver->stopData();
  sensor_has_started = 0;

  logInfo(2, "The sensor has been stoped");

  logInfo(1, "<< UskinSensor::StopSensor()");

  return 1;
};

// Get sensor's frame size
int UskinSensor::GetUskinFrameSize()
{
  logInfo(1, ">> UskinSensor::GetUskinFrameSize()");
  logInfo(1, "<< UskinSensor::GetUskinFrameSize()");
  return frame_size;
};

// Calibrate sensor. Sensor must be at rest during the this execution
void UskinSensor::CalibrateSensor()
{
  logInfo(1, ">> UskinSensor::CalibrateSensor()");

  logInfo(2, "Please do not touch the sensor while it is being calibrated!");

  // Check if sensor has been started
  if (!get_sensor_status())
  {
    logError(2, "You must start the sensor first!!");

    logInfo(1, "<< UskinSensor::CalibrateSensor()");

    return;
  }

  // If sensor has been calibrated before, structures have already been created
  if (get_sensor_calibration_status())
  {
    // Disabling the flag so that data retrieved is not normalized
    sensor_is_calibrated = 0;

    retrieveSensorMinReadings(10); // Retrieve minimum values out of 10 frame readings

    sensor_is_calibrated = 1;
  }
  else // first time calibrating the sensor, structures need to be allocated
  {
    frame_min_reads_size = frame_size;
    frame_min_reads = (unsigned long int **)std::malloc(frame_min_reads_size * sizeof(unsigned long int *));

    for (int i = 0; i < frame_min_reads_size; i++)
    {
      frame_min_reads[i] = new unsigned long int[3]{65000, 65000, 65000};
    }

    retrieveSensorMinReadings(10);
  }

  sensor_is_calibrated = 1;
  logInfo(1, "<< UskinSensor::CalibrateSensor()");
};

unsigned long int ** UskinSensor::getCalibrationValues()
{
  return frame_min_reads;
};


// Read and store latest sensor's frame reading. It will be stored at uskinCanDrive.frame_reading
void UskinSensor::RetrieveFrameData()
{
  logInfo(1, ">> UskinSensor::GetFrameData_xyzValues()");

  // to store raw can_frame readings
  struct can_frame *raw_data[frame_size];
  int n_frames_read;

  // frame_reading->clear();

  if (!sensor_has_started)
  {
    logError(2, "You must start the sensor first!!");
    return;
  }

  n_frames_read = driver->readData(raw_data, frame_size, convertIndextoCanID(frame_size - 1));

  for (int i = 0; i < n_frames_read; i++)
  {
    // Convert and store raw can_frame data
    int index = convertCanIDtoIndex(raw_data[i]->can_id);
    storeNodeReading(&frame_reading->instant_reading[index], raw_data[i], index);
  }

  // Attach a timestamp to data
  gettimeofday(&frame_reading->timestamp, NULL);

  // Save data if CSV file has been opened, otherwise just print it in log file
  SaveData();

  logInfo(1, "<< UskinSensor::GetFrameData_xyzValues()");

  return;
};

// Get x, y and z displacement readings for a single node
_uskin_node_time_unit_reading *UskinSensor::GetNodeData_xyzValues(int node)
{
  logInfo(1, ">> UskinSensor::GetNodeData_xyzValues(" + std::to_string(node) + ")");

  if (node >= frame_size)
  {
    logError(2, "Specified node index is to high. uSkin frame_size is" + std::to_string(frame_size));
    return NULL;
  }

  logInfo(1, "<< UskinSensor::GetNodeData_xyzValues(" + std::to_string(node) + ")");

  // logInfo(2, "================================================" + frame_reading->instant_reading[node].to_str());

  return &frame_reading->instant_reading[node];
};

_uskin_node_time_unit_reading *UskinSensor::GetFrameData()
{
  logInfo(1, ">> UskinSensor::GetFrameData()");

  return frame_reading->instant_reading;
  logInfo(1, "<< UskinSensor::GetFrameData()");
}
// uskin_time_unit_reading *UskinSensor::GetNodeData_yValues(int node){}; // TODO
// uskin_time_unit_reading *UskinSensor::GetNodeData_zValues(int node){}; // TODO

// Print frame reading contents
void UskinSensor::PrintData()
{
  logInfo(3, "Message recieved at " + std::string(asctime(gmtime(&frame_reading->timestamp.tv_sec))) + "." + std::to_string(frame_reading->timestamp.tv_usec) + "\n");

  for (int i = 0; i < frame_size; i++)
  {
    std::stringstream message;

    message << "CAN ID: " << std::hex << frame_reading->instant_reading[i].node_id << std::dec << "  X:  " << frame_reading->instant_reading[i].x_value << "  Y:  " << frame_reading->instant_reading[i].y_value << "  Z:  " << frame_reading->instant_reading[i].z_value << std::endl;
    logInfo(4, message.str());
  }

  return;
}

// Print frame reading contents
void UskinSensor::PrintNormalizedData()
{
  logInfo(3, "Message recieved at " + std::string(asctime(gmtime(&frame_reading->timestamp.tv_sec))) + "." + std::to_string(frame_reading->timestamp.tv_usec) + "\n");

  for (int i = 0; i < frame_size; i++)
  {
    std::stringstream message;

    message << "CAN ID: " << std::hex << frame_reading->instant_reading[i].node_id << std::dec << "  X:  " << frame_reading->instant_reading[i].x_value_normalized << "  Y:  " << frame_reading->instant_reading[i].y_value_normalized << "  Z:  " << frame_reading->instant_reading[i].z_value_normalized << std::endl;
    logInfo(4, message.str());
  }

  return;
}

// Store retrieved data in existing CSV file.
void UskinSensor::SaveData()
{
  logInfo(1, ">> UskinSensor::SaveData()");

  if (data_is_being_saved) // Data is already being saved
  {
    struct tm *timeinfo;
    char time_str[20];

    // Append reading timestamp
    timeinfo = localtime(&frame_reading->timestamp.tv_sec);
    strftime(time_str, 20, "%F_%T", timeinfo);
    csv_file << std::string(time_str) << "." << std::to_string(frame_reading->timestamp.tv_usec);

    // Append readings
    for (int i = 0; i < frame_size; i++)
    {
      csv_file << "," << std::hex << frame_reading->instant_reading[i].node_id << std::dec << "," << frame_reading->instant_reading[i].x_value << "," << frame_reading->instant_reading[i].y_value << "," << frame_reading->instant_reading[i].z_value;
    }

    csv_file << std::endl;

    logInfo(2, "Data has been recorded");
    PrintData();
  }
  else // No csv file was opened, printing the data to log file
  {
    logInfo(2, "No CSV file has been opened yet!");
    PrintData();
  }

  logInfo(1, "<< UskinSensor::SaveData()");

  return;
}

// Open new CSV file and initialize data structure. Filename must be the include the file name and full path
void UskinSensor::SaveData(std::string filename)
{
  logInfo(1, ">> UskinSensor::SaveData(" + filename + ")");

  if (!data_is_being_saved)
  {
    time_t timer;
    struct tm *timeinfo;
    char csv_name[20];
    time(&timer);
    timeinfo = localtime(&timer);
    strftime(csv_name, 20, "%F_%T", timeinfo);

    //printf("%s, %ld \n", filename.c_str(), filename.size());

    // if (!filename.compare(filename.size() - 3, 4, ".csv"))
    // {
    //   logError(3, "Filename must be of .csv type");
    //   return;
    // }

    // Open file with provided filename and timestamp
    csv_file.open(filename + "_" + std::string(csv_name) + ".csv");

    initializeCSVdataStructure(&csv_file);

    data_is_being_saved = 1;
  }
  else
  {
    logError(2, "A CSV file has already been opened");
  }

  logInfo(1, "<< UskinSensor::SaveData()");

  return;
}

// Store normalized data in existing CSV file.
void UskinSensor::SaveNormalizedData()
{
  logInfo(1, ">> UskinSensor::SaveNormalizedData()");

  if (normalized_data_is_being_saved) // Data is already being saved
  {
    struct tm *timeinfo;
    char time_str[20];

    // Append reading timestamp
    timeinfo = localtime(&frame_reading->timestamp.tv_sec);
    strftime(time_str, 20, "%F_%T", timeinfo);
    normalized_csv_file << std::string(time_str) << "." << std::to_string(frame_reading->timestamp.tv_usec);

    // Append readings
    for (int i = 0; i < frame_size; i++)
    {
      normalized_csv_file << "," << std::hex << frame_reading->instant_reading[i].node_id << std::dec << "," << frame_reading->instant_reading[i].x_value_normalized << "," << frame_reading->instant_reading[i].y_value_normalized << "," << frame_reading->instant_reading[i].z_value_normalized;
    }

    normalized_csv_file << std::endl;

    logInfo(2, "Data has been recorded");
    PrintNormalizedData();
  }
  else // No csv file was opened, printing the data to log file
  {
    logInfo(2, "No CSV file has been opened yet!");
    PrintNormalizedData();
  }

  logInfo(1, "<< UskinSensor::SaveNormalizedData()");

  return;
}

// Open new CSV file and initialize data structure. Filename must be the include the file name and full path
void UskinSensor::SaveNormalizedData(std::string filename)
{
  logInfo(1, ">> UskinSensor::SaveNormalizedData(" + filename + ")");

  if (!normalized_data_is_being_saved)
  {
    time_t timer;
    struct tm *timeinfo;
    char csv_name[20];
    time(&timer);
    timeinfo = localtime(&timer);
    strftime(csv_name, 20, "%F_%T", timeinfo);

    //printf("%s, %ld \n", filename.c_str(), filename.size());

    // if (!filename.compare(filename.size() - 3, 4, ".csv"))
    // {
    //   logError(3, "Filename must be of .csv type");
    //   return;
    // }

    // Open file with provided filename and timestamp
    normalized_csv_file.open(filename + "_" + std::string(csv_name) + ".csv");

    initializeCSVdataStructure(&normalized_csv_file);

    normalized_data_is_being_saved = 1;
  }
  else
  {
    logError(2, "A CSV file has already been opened");
  }

  logInfo(1, "<< UskinSensor::SaveNormalizedData()");

  return;
}

// Check if sensor has started
bool UskinSensor::get_sensor_status()
{
  logInfo(1, ">> UskinSensor::get_sensor_status()");

  logInfo(1, "<< UskinSensor::get_sensor_status()");

  return sensor_has_started == 1 ? true : false;
}

// Check if sensor has been calibrated
bool UskinSensor::get_sensor_calibration_status()
{
  logInfo(1, ">> UskinSensor::get_sensor_calibration_status()");

  logInfo(1, "<< UskinSensor::get_sensor_calibration_status()");

  return sensor_is_calibrated == 1 ? true : false;
}

// Check if data is being saved in CSV file
bool UskinSensor::get_sensor_saved_data_status()
{
  logInfo(1, ">> UskinSensor::get_sensor_saved_data_status()");

  logInfo(1, "<< UskinSensor::get_sensor_saved_data_status()");

  return data_is_being_saved == 1 ? true : false;
}

// Store minimum x, y and z displacement readings among all sensitive nodes. These values are different for each node
void UskinSensor::retrieveSensorMinReadings(int number_of_readings)
{
  // number_of_readings is the number of sensor's frame readings we will evaluate
  for (int i = 0; i < number_of_readings; i++)
  {
    RetrieveFrameData();
    for (int i = 0; i < frame_min_reads_size; i++) // For each of the sensor's nodes
    {

      if (frame_reading->instant_reading[i].x_value < frame_min_reads[i][0])
        frame_min_reads[i][0] = frame_reading->instant_reading[i].x_value; //Minimum x value for node i
      if (frame_reading->instant_reading[i].y_value < frame_min_reads[i][1])
        frame_min_reads[i][1] = frame_reading->instant_reading[i].y_value; //Minimum y value for node i
      if (frame_reading->instant_reading[i].z_value < frame_min_reads[i][2])
        frame_min_reads[i][2] = frame_reading->instant_reading[i].z_value; //Minimum z value for node i
    }
  }
  return;
}

// Normalize data using MinMax strategy from values minimum readings acquired during calibration
bool UskinSensor::NormalizeData()
{
  logInfo(1, ">> UskinSensor::NormalizeData()");

  if (get_sensor_calibration_status()) // If sensor was calibrated, normalize values
  {
    logInfo(2, "Attempting to normalize uskin frame readings...");
    frame_reading->normalize();
    SaveNormalizedData();
    logInfo(1, "<< UskinSensor::NormalizeData()");
    return true;
  }

  logInfo(1, "<< UskinSensor::NormalizeData()");
  return false;
}

// Create necessary columns in CSV file
void UskinSensor::initializeCSVdataStructure(std::ofstream *csv)
{
  *csv << "Timestamp";

  for (int i = 0; i < frame_size; i++)
  {
    *csv << ",CAN ID, X Values,Y Values, Z Values";
  }
  *csv << std::endl;
}