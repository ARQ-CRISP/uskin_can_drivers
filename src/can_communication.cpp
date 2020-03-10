/*
 * Copyright: (C) 2019 CRISP, Advanced Robotics at Queen Mary,
 *                Queen Mary University of London, London, UK
 * Author: Rodrigo Neves Zenha <r.neveszenha@qmul.ac.uk>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */
/**
 * \file can_communication.cpp
 *
 * \author Rodrigo Neves Zenha
 * \copyright  Released under the terms of the GNU GPL v3.0.
 */

#include "../include/can_communication.h"

// CanDriver constructors: It is possible to provide CAN network and device CAN ID of the sensor
CanDriver::CanDriver(){};

CanDriver::CanDriver(std::string new_network)
{
    ifname.assign(new_network);
};
CanDriver::CanDriver(std::string new_network, __u32 new_device_id)
{
    ifname.assign(new_network);
    device_id = new_device_id;
}

CanDriver::CanDriver(__u32 new_device_id)
{
    device_id = new_device_id;
}

CanDriver::~CanDriver(){};

//###################### Utils #########################

// Loging utils
void logInfo(int identation_level, std::string info)
{
    if (DEBUG)
    {
        std::string spaces(2 * identation_level, ' ');
        std::cout << "INFO:  " << spaces << info << std::endl;
        return;
    }
}

void logError(int identation_level, std::string error)
{
    if (DEBUG)
    {
        std::string spaces(2 * identation_level, ' ');
        std::cout << "ERROR:  " << spaces << "!! " << error << " !!" << std::endl;
        return;
    }
}

// Converting Hex MSB and LSB (2 bytes) to Dec
unsigned int convert_16bit_hex_to_dec(__u8 *data)
{
    return long(data[0] << 8 | data[1]);
}

unsigned long convert_dec_to_24bit_hex(unsigned int data)
{
    return long((data / 256) * 100 + ((data % 256) / 16) * 10 + ((data % 256) % 16));
}

// Convert can_frame data structure to string
std::string canFrameToString(can_frame *message)
{
    std::stringstream converted_msg;

    converted_msg << "Received message from device with CAN ID: " << std::hex << message->can_id << "; With data: ";

    for (int i = 1; i < 7; i++) // Reading bytes 2-7 (1st and 8th bytes unused by the sensor's protocol)
        converted_msg << std::hex << int(message->data[i]) << " ";

    return converted_msg.str();
}

// Checks if two can_id's are consequetive to each other
bool checkMessagesIdOrder(canid_t current_can_id, canid_t previous_can_id)
{
    // Logic to check if previous_can_id precedes current_can_id
    if ((((current_can_id % 100) % 10) > ((previous_can_id % 100) % 10)) || ((((current_can_id % 100) % 10) == ((previous_can_id % 100) % 10)) && ((current_can_id % 100) > (previous_can_id % 100))))
    {
        return true;
    }
    return false;
}

//###################### CanDriver #########################
// Bind connection to the sensor
int CanDriver::openConnection()
{

    logInfo(1, ">> CanDriver::open_connection()");

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        logError(2, "Error while opening socket");
        logInfo(1, "<< CanDriver::open_connection()");

        return 0;
    }

    strcpy(ifr.ifr_name, (char *)ifname.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    logInfo(2, "Connecting to " + ifname + " at index " + std::to_string(ifr.ifr_ifindex));

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        logError(2, "Error in socket bind");
        logInfo(1, "<< CanDriver::open_connection()");

        return 0;
    }

    int sock_buf_size;
    int i = sizeof(sock_buf_size);
    int nbytes = getsockopt(s, SOL_SOCKET, SO_RCVBUF,
                            &sock_buf_size, (socklen_t *)&i);

    logInfo(2, "current sock_buf_size" + std::to_string(sock_buf_size));

    sock_buf_size = 0x80000;

    nbytes = setsockopt(s, SOL_SOCKET, SO_RCVBUF,
                        &sock_buf_size, sizeof(sock_buf_size));

    i = sizeof(sock_buf_size);
    nbytes = getsockopt(s, SOL_SOCKET, SO_RCVBUF,
                        &sock_buf_size, (socklen_t *)&i);

    logInfo(2, "new current sock_buf_size" + std::to_string(sock_buf_size));

    logInfo(1, "<< CanDriver::open_connection()");

    return 1;
}

// Send message to the sensor
int CanDriver::sendMessage(can_frame sending_frame)
{
    int return_value = 1;

    logInfo(2, ">> CanDriver::send_message()");

    int nbytes;

    nbytes = write(s, &sending_frame, sizeof(struct can_frame));

    logInfo(3, "Sent " + std::to_string(nbytes) + " bytes");

    if (nbytes < 0)
    {
        logError(3, "No data was sent, possible problems with connection");
        return_value = 0;
    }
    logInfo(2, "<< CanDriver::send_message()");

    return return_value;
}

// Read message from the sensor
int CanDriver::readMessage(can_frame *receiving_frame)
{
    logInfo(2, ">> CanDriver::read_message()");
    int nbytes;
    struct sockaddr_can addr;

    socklen_t len = sizeof(addr);

    //nbytes = read(s, receiving_frame, sizeof(struct can_frame));
    nbytes = recvfrom(s, receiving_frame, sizeof(struct can_frame),
                      0, (struct sockaddr *)&addr, &len);

    struct timeval tv;
    ioctl(s, SIOCGSTAMP, &tv);
    logInfo(3, ">> Reading at: " + std::to_string(tv.tv_sec) + "." + std::to_string(tv.tv_usec));

    if (nbytes < 0)
    {
        logError(3, "Error while reading raw socket");
        logInfo(2, "<< CanDriver::read_message(-1)");

        return 0;
    }

    /* paranoid check ... */
    if (nbytes < sizeof(struct can_frame))
    {
        logError(3, "read: incomplete CAN frame");
        logInfo(2, "<< CanDriver::read_message(-2)");

        return 0;
    }

    logInfo(2, canFrameToString(receiving_frame));

    logInfo(2, "<< CanDriver::read_message()");

    return 1;
}

// Request the sensor to start reading data
int CanDriver::requestData()
{
    int return_value = 1;
    logInfo(1, ">> CanDriver::request_data()");

    struct can_frame sending_frame;

    sending_frame.can_id = device_id;
    sending_frame.can_dlc = 2; /* frame payload length in byte (0 .. 8) */
    sending_frame.data[0] = 0x07;
    sending_frame.data[1] = 0x00;

    if (!sendMessage(sending_frame))
    {
        return_value = 0;
        logError(2, "<< Problems Requesting Data");
    }
    else
    {
        data_requested = true;
    }

    logInfo(1, "<< CanDriver::request_data()");

    return return_value;
}

// Request the sensor to stop reading data
void CanDriver::stopData()
{
    logInfo(1, ">> CanDriver::stop_data()");
    struct can_frame sending_frame;

    sending_frame.can_id = device_id;
    sending_frame.can_dlc = 2; /* frame payload length in byte (0 .. 8) */
    sending_frame.data[0] = 0x07;
    sending_frame.data[1] = 0x01;

    sendMessage(sending_frame);

    CanDriver::data_requested = false;

    logInfo(1, "<< CanDriver::stop_data()");

    return;
}

// Read a stream of data form the sensor. Stream lenght is defined by frame_size
int CanDriver::readData(can_frame **receiving_frame, int frame_size, int max_can_ID)
{
    logInfo(1, ">> CanDriver::read_data()");
    int retreived_elements = 0;

    logInfo(2, "Reading data with frame size " + std::to_string(frame_size));

    if (!data_requested) // check if data has already been requested
    {
        logError(2, "You must first request data from the sensor");
        logInfo(1, "<< CanDriver::read_data()");
        return 0;
    }

    if (is_filter_set) // reset socket options if no filter is defined
    {
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
        is_filter_set = false;
        logInfo(2, "New filter now set.");
    }

    
    if (temporary_reading_available)
    {
        receiving_frame[0] = new can_frame;
        *receiving_frame[0] = temporary_reading;
        temporary_reading_available = false;
        retreived_elements++;
    }

    // if (!CanDriver::readMessage(receiving_frame[0]))
    // {
    //     logError(2, "Problems reading data");
    //     free(receiving_frame[0]);
    //     return retreived_elements;
    // }

    

    // Check if first message read has right ID
    // while (convert_dec_to_24bit_hex(receiving_frame[0]->can_id) != 100)
    // {
    //     logError(2, "The expected message has the wrong ID");
    //     // Keep searching for the right message ID
    //     if (!CanDriver::readMessage(receiving_frame[0]))
    //     {
    //         logError(2, "Problems reading data");
    //     }
    // }

    for (int i = retreived_elements; i < frame_size; i++) // frame_size is number of can frames the users wants to read before completion
    {
        receiving_frame[i] = new can_frame;
        if (!CanDriver::readMessage(receiving_frame[i]))
        {
            logError(2, "Problems reading data");
            free(receiving_frame[i]);
            return retreived_elements;
        }

        if (i > 0 && !checkMessagesIdOrder(convert_dec_to_24bit_hex(receiving_frame[i]->can_id), convert_dec_to_24bit_hex(receiving_frame[i - 1]->can_id)))
        {
            logError(2, "Problems with messages order");
            temporary_reading = *receiving_frame[i];
            temporary_reading_available = true;
            free(receiving_frame[i]);
            // for (int j = 0; j <= i; j++)
            // {
            //     delete receiving_frame[j];
            // }

            // readData(receiving_frame, frame_size);
            break;
        }
        retreived_elements++;

        if (convert_dec_to_24bit_hex(receiving_frame[i]->can_id) == max_can_ID)
        {
            logInfo(2, "Message with Maximum CanID has been received ");
            break;
        }
    }

    logInfo(1, "<< CanDriver::read_data()");

    return retreived_elements;
}

// Read a filtered stream of data form the sensor. Stream lenght is defined by frame_size.
/* can_frame **CanDriver::readData(int number_of_filters, struct can_filter *rfilter)
{
    logInfo(1, ">> CanDriver::read_data()");

    struct can_frame *receiving_frame[number_of_filters];

    if (!data_requested)
    {
        logError(2, "You must first request data from the sensor");
        logInfo(1, "<< CanDriver::read_data()");
        return NULL;
    }

    // set filter independently of state of is_filter_set
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, number_of_filters);
    is_filter_set = true;

    for (int i = 0; i < number_of_filters; i++)
    {
        receiving_frame[i] = new can_frame;
        if (!CanDriver::readMessage(receiving_frame[i]))
        {
            logError(2, "!! Problems reading data !!");
        }
        else
            logInfo(2, canFrameToString(receiving_frame[i]));
    }

    logInfo(1, "<< CanDriver::read_data()");

    return receiving_frame;
} */
