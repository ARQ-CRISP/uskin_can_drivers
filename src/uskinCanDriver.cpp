#include <string>
#include <iostream>
#include "../include/uskinCanDriver.h"

unsigned int convert_16bit_hex_to_dec(__u8 *data)
{
  // We will always be converting Hex MSB and LSB (2 bytes) to Dec
  return long(data[0] << 8 | data[1]);
}

void storeNodeReading(struct _uskin_node_time_unit_reading *node_reading, struct can_frame *raw_node_reading, int sequence)
{
  node_reading->node_id = raw_node_reading->can_id;
  node_reading->sequence = sequence;
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

void UskinSensor::retrieveSensorMinMaxReadings(int number_of_readings)
{
  for (int i = 0; i < number_of_readings; i++) // We'll read 10 frames of the sensor and save the minium value acuqired for each node
    {
      RetrieveFrameData();
      for (int i = 0; i < frame_min_reads_size; i++)
      {

        if (frame_reading->instant_reading[i].x_value < frame_min_reads[i][0])
          frame_min_reads[i][0] = frame_reading->instant_reading[i].x_value; //Minimum x value for node i
        if (frame_reading->instant_reading[i].y_value < frame_min_reads[i][1])
          frame_min_reads[i][1] = frame_reading->instant_reading[i].y_value; //Minimum y value for node i
        if (frame_reading->instant_reading[i].z_value < frame_min_reads[i][2])
          frame_min_reads[i][2] = frame_reading->instant_reading[i].z_value; //Minimum z value for node i

        /* if (frame_reading->instant_reading[i].x_value > frame_max_reads[i][0])
          frame_min_reads[i][0] = frame_reading->instant_reading[i].x_value; //Maximum x value for node i
        if (frame_reading->instant_reading[i].y_value > frame_max_reads[i][1])
          frame_min_reads[i][1] = frame_reading->instant_reading[i].y_value; //Maximum y value for node i
        if (frame_reading->instant_reading[i].z_value > frame_max_reads[i][2])
          frame_min_reads[i][2] = frame_reading->instant_reading[i].z_value; //Maximum z value for node i */
      }
    }
  return;
}

UskinSensor::UskinSensor() : frame_columns(USKIN_COLUMNS), frame_rows(USKIN_ROWS), frame_size(USKIN_COLUMNS * USKIN_ROWS)
{
  if (DEBUG)
    freopen((char *)log_file.c_str(), "w", stdout);

  driver = new CanDriver;

  frame_reading = new uskin_time_unit_reading;
  frame_reading->instant_reading = new struct _uskin_node_time_unit_reading[frame_size];

  return;
};

UskinSensor::UskinSensor(int column_nodes, int row_nodes) : frame_columns(column_nodes), frame_rows(row_nodes), frame_size(column_nodes * row_nodes)
{
  if (DEBUG)
    freopen((char *)log_file.c_str(), "w", stdout);

  driver = new CanDriver;

  frame_reading = new uskin_time_unit_reading;
  frame_reading->instant_reading = new struct _uskin_node_time_unit_reading[frame_size];

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

void UskinSensor::CalibrateSensor() // Leaving the sensor untouched for a period of time
{
  logInfo(1, ">> UskinSensor::CalibrateSensor()");

  logInfo(2, "Please do not touch the sensor while it is being calibrated!");

  if (!get_sensor_status())
  {
    logError(2, "You must start the sensor first!!");

    logInfo(1, "<< UskinSensor::CalibrateSensor()");

    return;
  }

  if (get_sensor_calibration_status()) // Sensor has already been calibrated once, so structures have already been created
  {
    sensor_is_calibrated = 0; // Start by disabling the flag so that data retrieved is not normalized

    retrieveSensorMinMaxReadings(10);

    sensor_is_calibrated = 1;
  }
  else // first time calibrating the sensor, structures need to be allocated
  {
    frame_min_reads_size = frame_size;
    frame_min_reads = (unsigned long int **)std::malloc(frame_min_reads_size * sizeof(unsigned long int *));
    //frame_max_reads = (unsigned long int **)std::malloc(frame_size * sizeof(unsigned long int *));

    for (int i = 0; i < frame_min_reads_size; i++)
    {
      frame_min_reads[i] = new unsigned long int[3]{65000, 65000, 65000};
      //frame_max_reads[i] = new unsigned long int[3]();
    }

    retrieveSensorMinMaxReadings(10);

  }

  sensor_is_calibrated = 1;
  // Improve min values per node
  //float norm = (((float)reading->back().z_data - 18300)/ (25500 - 18300))*100;
  logInfo(1, "<< UskinSensor::CalibrateSensor()");
};

void UskinSensor::RetrieveFrameData()
{
  logInfo(1, ">> UskinSensor::GetFrameData_xyzValues()");

  struct can_frame *raw_data[frame_size];

  frame_reading->clear();

  if (!sensor_has_started)
  {
    logError(2, "You must start the sensor first!!");
    return;
  }

  driver->readData(raw_data, frame_size);

  for (int i = 0; i < frame_size; i++)
  {
    storeNodeReading(&frame_reading->instant_reading[i], raw_data[i], i);
  }

  frame_reading->number_of_nodes = frame_size;
  gettimeofday(&frame_reading->timestamp, NULL);

  if (get_sensor_calibration_status()) // If sensor was calibrated, normalize values
  {
    logInfo(2, "Attempting to normalize uskin frame readings...");
    frame_reading->normalize();
  }

  SaveData();

  logInfo(1, "<< UskinSensor::GetFrameData_xyzValues()");

  return;
};

//uskin_time_unit_reading *UskinSensor::GetFrameData_xyzValues(){};

_uskin_node_time_unit_reading *UskinSensor::GetNodeData_xyzValues(int node)
{
  logInfo(1, ">> UskinSensor::GetNodeData_xyzValues(" + std::to_string(node) + ")");

  if (node >= frame_size)
  {
    logError(2, "Specified node index is to high. uSkin frame_size is" + std::to_string(frame_size));
    return NULL;
  }

  logInfo(1, "<< UskinSensor::GetNodeData_xyzValues(" + std::to_string(node) + ")");

  return &frame_reading->instant_reading[node];
};

// uskin_time_unit_reading *UskinSensor::GetFrameData_xValues(){};

// uskin_time_unit_reading *UskinSensor::GetFrameData_yValues(){};

// uskin_time_unit_reading *UskinSensor::GetFrameData_zValues(){};

uskin_time_unit_reading *UskinSensor::GetNodeData_xValues(int node){}; // TODO
uskin_time_unit_reading *UskinSensor::GetNodeData_yValues(int node){}; // TODO
uskin_time_unit_reading *UskinSensor::GetNodeData_zValues(int node){}; // TODO

void UskinSensor::PrintData()
{
  logInfo(3, "Message recieved at " + std::string(asctime(gmtime(&frame_reading->timestamp.tv_sec))) + "." + std::to_string(frame_reading->timestamp.tv_usec) + "\n");

  for (int i = 0; i < frame_reading->number_of_nodes; i++)
  {
    std::stringstream message;

    message << "CAN ID: " << std::hex << frame_reading->instant_reading[i].node_id << std::dec << "  X:  " << frame_reading->instant_reading[i].x_value << "  Y:  " << frame_reading->instant_reading[i].y_value << "  Z:  " << frame_reading->instant_reading[i].z_value << std::endl;
    logInfo(4, message.str());
  }

  return;
}

void UskinSensor::SaveData()
{
  logInfo(1, ">> UskinSensor::SaveData()");

  if (data_is_being_saved) // Data is already being saved
  {
    struct tm *timeinfo;
    char time_str[20];

    timeinfo = localtime(&frame_reading->timestamp.tv_sec);
    strftime(time_str, 20, "%F_%T", timeinfo);
    csv_file << std::string(time_str) << "." << std::to_string(frame_reading->timestamp.tv_usec);

    for (int i = 0; i < frame_reading->number_of_nodes; i++)
    {
      csv_file << "," << std::hex << frame_reading->instant_reading[i].node_id << std::dec << "," << frame_reading->instant_reading[i].x_value << "," << frame_reading->instant_reading[i].y_value << "," << frame_reading->instant_reading[i].z_value;
    }

    csv_file << std::endl;

    logInfo(2, "Data has been recorded");

  }
  /*   else // No csv file was opened, printing the data to log file
  {
    PrintData();
  } */
  else
  {
    logError(2, "No CSV file has been opened yet!");
  }

  logInfo(1, "<< UskinSensor::SaveData()");

  return;
}

void UskinSensor::SaveData(std::string filename)
{
  logInfo(1, ">> UskinSensor::SaveData()");

  if (!data_is_being_saved)
  {
    //printf("%s, %ld \n", filename.c_str(), filename.size());

    if (!filename.compare(filename.size() - 3, 4, ".csv"))
    {
      logError(3, "Filename must be of .csv type");
      return;
    }

    csv_file.open("/home/rodrigo/Documents/github/uskin_publisher/src/uskin_ros_publisher/csv_files/" + filename);

    csv_file << "Timestamp";

    for (int i = 0; i < frame_size; i++)
    {
      csv_file << ",CAN ID, X Values,Y Values, Z Values";
    }
    csv_file << std::endl;

    data_is_being_saved = 1;
  }

  logInfo(1, "<< UskinSensor::SaveData()");

  return;
}

bool UskinSensor::get_sensor_status()
{
  logInfo(1, ">> UskinSensor::get_sensor_status()");

  logInfo(1, "<< UskinSensor::get_sensor_status()");

  return sensor_has_started == 1 ? true : false;
}

bool UskinSensor::get_sensor_calibration_status()
{
  logInfo(1, ">> UskinSensor::get_sensor_calibration_status()");

  logInfo(1, "<< UskinSensor::get_sensor_calibration_status()");

  return sensor_is_calibrated == 1 ? true : false;
}

bool UskinSensor::get_sensor_saved_data_status()
{
  logInfo(1, ">> UskinSensor::get_sensor_saved_data_status()");

  logInfo(1, "<< UskinSensor::get_sensor_saved_data_status()");

  return data_is_being_saved == 1 ? true : false;
}
