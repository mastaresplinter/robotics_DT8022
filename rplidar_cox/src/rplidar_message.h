#ifndef RPLIDAR_MESSAGE_H
#define RPLIDAR_MESSAGE_H

#include <netinet/in.h>

typedef struct {
    uint16_t angle;
    uint16_t dist_mm;
    uint8_t quality;
    uint8_t flag;
}lidar_response_message_t;


#endif