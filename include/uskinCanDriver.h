#ifndef USKINCANDRIVER_H
#define USKINCANDRIVER_H

#include <iostream>
#include <array>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <vector>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define DEBUG 0

void input_log(std::string message);

void output_log(std::string message);

unsigned int convert_16bit_hex_to_dec(__u8 * data);

unsigned long convert_32bit_hex_to_dec(__u32 * data);

struct _single_node_reading
{
  __u32 id;
  unsigned int x_data;
  unsigned int y_data;
  unsigned int z_data;
};
/* typedef struct uksin_x_values
{
  std::array<unsigned int, > x_val:
} Uskin_x_values;

typedef struct uskin_matrix{
  unsigned int 
}  */

class UskinCanDriver
{
  // Access specifier
  private:
    int s;
	  struct sockaddr_can addr;
	  struct ifreq ifr;

    const char *ifname = "can0";

    bool data_requested = false;

    bool is_filter_set = false; // check if there is any filter applied to the socket (incoming data)

    void send_message(can_frame sending_frame);
    int read_message(can_frame * receiving_frame);


  public:
    UskinCanDriver();
    ~UskinCanDriver();

    int open_connection();

    void request_data(__u32 can_id);

    void stop_data(__u32 can_id);
    void read_data(std::vector <struct _single_node_reading> * instant_reading);
    //void read_data(struct can_filter * rfilter, long number_of_filters);

    void get_data(unsigned int (&matrix_xyz_values)[4][6][3]);

    void get_data_x_values(unsigned int (&matrix_xyz_values)[4][6]);
    void get_data_y_values();
    void get_data_z_values();



};

#endif
