#ifndef LIDAR_MESSAGE_H
#define LIDAR_MESSAGE_H

#include <netinet/in.h>

#define MESSAGE_SIZE 5

typedef struct {
    uint8_t data[MESSAGE_SIZE]; 
}lidar_message_t;


#endif