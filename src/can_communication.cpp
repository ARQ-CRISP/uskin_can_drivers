#include "../include/can_communication.h"

CanDriver::CanDriver()
{
    if (DEBUG)
        freopen((char *)log_file.c_str(), "w", stdout);
};
CanDriver::CanDriver(std::string new_network)
{
    if (DEBUG)
        freopen((char *)log_file.c_str(), "w", stdout);

    ifname.assign(new_network);
};
CanDriver::CanDriver(std::string new_network, __u32 new_device_id)
{
    if (DEBUG)
        freopen((char *)log_file.c_str(), "w", stdout);

    ifname.assign(new_network);
    device_id = new_device_id;
}
CanDriver::CanDriver(__u32 new_device_id)
{
    if (DEBUG)
        freopen((char *)log_file.c_str(), "w", stdout);

    device_id = new_device_id;
}
CanDriver::~CanDriver(){};

//###################### UTILS #########################

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

std::string canFrameToString(can_frame *message)
{
    std::stringstream converted_msg;

    converted_msg << "Received message from device with CAN ID: " << std::hex << message->can_id << "; With data: ";

    for (int i = 1; i < 7; i++)
        converted_msg << std::hex << int(message->data[i]) << " ";

    return converted_msg.str();
}

//###################### CanDriver #########################
int CanDriver::openConnection()
{

    logInfo(1, ">> CanDriver::open_connection()");

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        logError(2, "Error while opening socket");
        logInfo(1, "<< CanDriver::open_connection(-1)");

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
        logInfo(1, "<< CanDriver::open_connection(-2)");

        return 0;
    }

    logInfo(1, "<< CanDriver::open_connection()");

    return 1;
}

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

int CanDriver::readMessage(can_frame *receiving_frame)
{
    logInfo(2, ">> CanDriver::read_message()");
    int nbytes;
    struct sockaddr_can addr;

    socklen_t len = sizeof(addr);

    //nbytes = read(s, receiving_frame, sizeof(struct can_frame));
    nbytes = recvfrom(s, receiving_frame, sizeof(struct can_frame),
                      0, (struct sockaddr *)&addr, &len);

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

    logInfo(2, "<< CanDriver::read_message()");

    return 1;
}

int CanDriver::requestData()
{
    int return_value = 1;
    logInfo(1, ">> CanDriver::request_data()");

    struct can_frame sending_frame;

    sending_frame.can_id = device_id;
    sending_frame.can_dlc = 2;
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

void CanDriver::stopData()
{
    logInfo(1, ">> CanDriver::stop_data()");
    struct can_frame sending_frame;

    sending_frame.can_id = device_id;
    sending_frame.can_dlc = 2;
    sending_frame.data[0] = 0x07;
    sending_frame.data[1] = 0x01;

    sendMessage(sending_frame);

    CanDriver::data_requested = false;

    logInfo(1, "<< CanDriver::stop_data()");

    return;
}

void CanDriver::readData(can_frame ** receiving_frame, int frame_size)
{
    logInfo(1, ">> CanDriver::read_data()");

    logInfo(2, "Reading data with frame size " + std::to_string(frame_size));

    if (!data_requested)
    {
        logError(2, "You must first request data from the sensor");
        logInfo(1, "<< CanDriver::read_data()");
        return;
    }

    if (is_filter_set) // reset socket options if no filter is defined
    {
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
        is_filter_set = false;
        logInfo(2, "New filter now set.");
    }

    for (int i = 0; i < frame_size; i++)
    {
        receiving_frame[i] = new can_frame;
        if (!CanDriver::readMessage(receiving_frame[i]))
        {
            logError(2, "Problems reading data");
        }
        /* else
            logInfo(2, canFrameToString(receiving_frame[i])); */
    }
    /* 
            struct _single_node_reading reading;
            reading.id = receiving_frame.can_id;
            //reading.id = convert_32bit_hex_to_dec((__u32*)&receiving_frame.can_id);
            reading.x_data = convert_16bit_hex_to_dec(&receiving_frame.data[1]);
            reading.y_data = convert_16bit_hex_to_dec(&receiving_frame.data[3]);
            reading.z_data = convert_16bit_hex_to_dec(&receiving_frame.data[5]);
 */

    logInfo(1, "<< CanDriver::read_data()");

    return;
}

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
