#include <string>
#include <iostream>
#include "uskinCanDriver.h"

UskinCanDriver::UskinCanDriver(){};
UskinCanDriver::~UskinCanDriver(){};


// Utilities
void input_log(std::string message)
{
  if(DEBUG)
    std::cout << "  >>" << message << '\n';
}

void output_log(std::string message)
{
  if(DEBUG)
    std::cout << "  <<" << message << '\n';
}


unsigned int convert_16bit_hex_to_dec(__u8 * data)
{
  // We will always be converting Hex MSB and LSB (2 bytes) to Dec 
  return long(data[0]<< 8 | data[1]);
}

unsigned long convert_32bit_hex_to_dec(__u32 * data)
{
  // We will always be converting Hex MSB and LSB (2 bytes) to Dec 
  return long(data[0]<< 16 | data[1]<<8 | data[2]);
}

void print_single_reading(struct _single_node_reading reading)
{
  printf("Reading from node with ID %X", reading.id );
  printf("x: %d \t y: %d \t z: %d\n", reading.x_data, reading.y_data, reading.z_data );

}

bool is_uskin_last_node(__u32 id)
{
  return id == 0x135;
}





int UskinCanDriver::open_connection()
{
  if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while opening socket");
    return -1;
  }

  strcpy(ifr.ifr_name, ifname);
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

  if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("Error in socket bind");
    return -2;
  }

  return 0;
}

void UskinCanDriver::send_message(can_frame sending_frame)
{

  int nbytes;

  nbytes = write(s, &sending_frame, sizeof(struct can_frame));

  printf("Sent %d bytes\n", nbytes);

  return;

}

int UskinCanDriver::read_message(can_frame * receiving_frame)
{

  int nbytes;
  struct sockaddr_can addr;

  socklen_t len = sizeof(addr);

  //nbytes = read(s, receiving_frame, sizeof(struct can_frame));
  nbytes = recvfrom(s, receiving_frame, sizeof(struct can_frame),
                  0, (struct sockaddr*)&addr, &len);
  
  if (nbytes < 0) {
    perror("can raw socket read");
    return -1;
  }

  /* paranoid check ... */
  if (nbytes < sizeof(struct can_frame)) {
    fprintf(stderr, "read: incomplete CAN frame\n");
    return -2;
  }

  return 1;
}


void UskinCanDriver::read_data(std::vector <struct _single_node_reading> * instant_reading)
{
  struct can_frame receiving_frame;

  
  input_log(" UskinCanDriver:read_data()");


  if (!UskinCanDriver::data_requested)
  {
    perror("You must first request data from the sensor");
    //return NULL;
  }
 
  if(is_filter_set) // reset socket options if no filter is defined 
  {
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    is_filter_set = false;

  }
  
  instant_reading->clear();

  while(instant_reading->size() < 24)
  {
    
    if(!UskinCanDriver::read_message(&receiving_frame))
      fprintf(stderr, "Problems reading data\n");
    else
    {
      struct _single_node_reading reading;
      reading.id = receiving_frame.can_id;
      //reading.id = convert_32bit_hex_to_dec((__u32*)&receiving_frame.can_id);
      reading.x_data = convert_16bit_hex_to_dec(&receiving_frame.data[1]);
      reading.y_data = convert_16bit_hex_to_dec(&receiving_frame.data[3]);
      reading.z_data = convert_16bit_hex_to_dec(&receiving_frame.data[5]);

      instant_reading->push_back(reading);
      
      //print_single_reading(reading);
      
    }
  }
   
  //printf("last can_id: %X: ", instant_reading.back().id);

  if (!is_uskin_last_node(instant_reading->back().id))
  {
      fprintf(stderr, "Problems reading matrix data instance\n");
      instant_reading->clear();
      //return NULL;
  }
  
  
  //printf("can_id: %X: ", receiving_frame.can_id);
  //for (int i = 0; i < 8; i++)
  //        printf("%02X ", receiving_frame.data[i]);
  //printf("\n");
  
  //printf("Converted value: %d\n", convert_16bit_hex_to_dec(&receiving_frame.data[5]));


  output_log(" UskinCanDriver:read_data()");

  return;

}


// Read data from a specified list of CAN ID's
/* void UskinCanDriver::read_data(struct can_filter * rfilter, long number_of_filters)
{
  struct can_frame receiving_frame;
  
  // set filter independently of state of is_filter_set
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, number_of_filters);
  is_filter_set = true;    

  
  input_log(" UskinCanDriver:read_data()");


  if (!UskinCanDriver::data_requested)
  {
    perror("You must first request data from the sensor");
    return;
  }

  if(!UskinCanDriver::read_message(&receiving_frame))
    fprintf(stderr, "Problems reading data\n");


  printf("can_id: %X: ", receiving_frame.can_id);
  for (int i = 0; i < 8; i++)
          printf("%02X ", receiving_frame.data[i]);
  printf("\n");

  printf("Converted value: %d\n", convert_16bit_hex_to_dec(&receiving_frame.data[5]));
  

  output_log(" UskinCanDriver:read_data()");

  return;

}
 */


void UskinCanDriver::request_data(__u32 can_id = 0x201)
{
  struct can_frame sending_frame;

  sending_frame.can_id  = (canid_t) can_id;
  sending_frame.can_dlc = 2;
  sending_frame.data[0] = 0x07;
  sending_frame.data[1] = 0x00;

  send_message(sending_frame);

  UskinCanDriver::data_requested = true;

  return;

}

void UskinCanDriver::stop_data(__u32 can_id = 0x201)
{
  struct can_frame sending_frame;

  sending_frame.can_id  = (canid_t) can_id;
  sending_frame.can_dlc = 2;
  sending_frame.data[0] = 0x07;
  sending_frame.data[1] = 0x01;

  send_message(sending_frame);

  UskinCanDriver::data_requested = false;

  return;

}


void UskinCanDriver::get_data(unsigned int (&matrix_xyz_values)[4][6][3])
{
  //get_data_x_values(matrix_xyz_values[0][0][0]);

return;
}

/* void UskinCanDriver::get_data_x_values(unsigned int (&matrix_xyz_values)[4][6]);
void UskinCanDriver::get_data_y_values();
void UskinCanDriver::get_data_z_values(); */
