#ifdef __cplusplus
extern "C"
{
#endif

#ifndef RPLIDAR_H
#define RPLIDAR_H

#include "server_rp.h"
#include <string.h>
#include <time.h>
#include <pthread.h>

#define BUFFER_SIZE 200
#define MESSAGE_SIZE 5
#define BUFFER_BYTES (BUFFER_SIZE * MESSAGE_SIZE)

#define ACTIVE 999
#define DISABLED 333

    // global circular buffer for storing messages
    extern char messages[BUFFER_BYTES];
    // lidar_message_t messages[BUFFER_SIZE];
    extern volatile int message_count;  // Global var to count number of messages
    extern volatile int message_index;  // Global var to keep track of message index
    extern pthread_mutex_t buff_mutex;  // mutex to be able to handle the global buffer from different threads
    extern pthread_mutex_t cox_mutex;
    extern volatile int cox_called_flag;

    void *flush_buffer(void *arg);
    void read_data_test(int *count);

#endif

#ifdef __cplusplus
}
#endif