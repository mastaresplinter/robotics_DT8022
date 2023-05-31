#include "rplidar.h"
#include "stop_vars.h"
// global buffer for storing messages
char messages[BUFFER_BYTES];
volatile int message_count = 0; // Global var to count number of messages
volatile int message_index = 0; // Global var to keep track of message index
volatile int global_stop_msg = DISABLED;

// mutex for the buffer
pthread_mutex_t buff_mutex = PTHREAD_MUTEX_INITIALIZER;

void *flush_buffer(void *arg)
{
    server_rp *server = (server_rp *)arg;

    while (global_stop_msg != ACTIVE)
    {
        char header[MESSAGE_SIZE] = {0};
        char buffer[MESSAGE_SIZE] = {0};
        while (read(server->new_socket, header, sizeof(header)) == MESSAGE_SIZE)
        {
            if ((__u_char)header[0] == 0xA5)
            {
                if (read(server->new_socket, buffer, sizeof(buffer)) == MESSAGE_SIZE)
                {
                    if (cox_called_flag == ACTIVE)
                    {
                        if (buffer[0] >> 2 != 0)
                        {
                            // add message to buffer
                            pthread_mutex_lock(&buff_mutex);
                            memcpy(&messages[message_index], buffer, MESSAGE_SIZE);
                            message_index = (message_index + MESSAGE_SIZE);
                            message_count++;
                            if (message_count == 200)
                                cox_called_flag = DISABLED;
                            pthread_mutex_unlock(&buff_mutex);
                        }
                    }
                }
            }
        }
    }
    return NULL;
}

void read_data_test(int *count)
{
    uint16_t start, quality, angle, distance;
    pthread_mutex_lock(&buff_mutex);
    int i, j;
    for (i = 0, j = message_index; i < message_count; i++, j = (j + MESSAGE_SIZE) % BUFFER_BYTES)
    {
        start = messages[j] & 1;
        quality = messages[j] >> 2;
        angle = ((messages[j + 1] >> 1) + (messages[j + 2] << 8)) >> 7;
        distance = ((messages[j + 3]) + (messages[j + 4] << 8)) >> 2;
        printf("Start: %d, Quality: %d, Angle: %d, Dist: %d\n", start, quality, angle, distance);
    }
    pthread_mutex_unlock(&buff_mutex);
    *count = *count + 1;
}