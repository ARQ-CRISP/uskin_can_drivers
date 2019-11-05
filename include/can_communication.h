#ifndef CANCOMMUNICATION_H
#define CANCOMMUNICATION_H

#include <iostream>
#include <sstream>
#include <cstdio>

#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define DEBUG 1

//###################### UTILS #########################

void logInfo(int identation_level, std::string info);

void logError(int identation_level, std::string error);

std::string canFrameToString(can_frame *message);

//###################### CanDriver #########################

class CanDriver
{
    // Access specifier
private:
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    std::string log_file = "/home/rodrigo/Documents/github/uskin_publisher/src/uskin_ros_publisher/log_files/CanDriver_log.output";

    std::string ifname = "can0"; // By default

    __u32 device_id = 0x201; // By default

    bool data_requested = false;

    bool is_filter_set = false; // check if there is any filter applied to the socket (incoming data)

    int sendMessage(can_frame sending_frame);
    int readMessage(can_frame *receiving_frame);

public:
    CanDriver();
    CanDriver(std::string new_network);
    CanDriver(std::string new_network, __u32 new_device_id);
    CanDriver(__u32 new_device_id);
    ~CanDriver();

    int openConnection();

    int requestData();

    void stopData();

    void readData(can_frame ** receiving_frame, int frame_size);

    //can_frame ** readData(int number_of_filters, struct can_filter *rfilter);
};

#endif