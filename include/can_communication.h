/*
 * Copyright: (C) 2019 CRISP, Advanced Robotics at Queen Mary,
 *                Queen Mary University of London, London, UK
 * Author: Rodrigo Neves Zenha <r.neveszenha@qmul.ac.uk>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */
/**
 * \file can_communication.h
 *
 * \author Rodrigo Neves Zenha
 * \copyright  Released under the terms of the GNU GPL v3.0.
 */

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

#define DEBUG 0

//###################### Utils #########################

void logInfo(int identation_level, std::string info);

void logError(int identation_level, std::string error);

unsigned int convert_16bit_hex_to_dec(__u8 *data);

unsigned long convert_dec_to_24bit_hex(unsigned int data);

std::string canFrameToString(can_frame *message);

//###################### CanDriver #########################

class CanDriver
{
    // Access specifier
private:
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    std::string ifname = "can0"; // Default network name

    __u32 device_id = 0x201; // Default device CAN ID

    bool data_requested = false; // Flags if data has already been requested

    bool is_filter_set = false; // Flags if there is any filter applied to the socket (for incoming data)

    can_frame temporary_reading;
    bool temporary_reading_available = false;

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

    int readData(can_frame **receiving_frame, int frame_size, int max_can_ID);

    //can_frame ** readData(int number_of_filters, struct can_filter *rfilter); // This method will allow to filter incoming data
};

#endif