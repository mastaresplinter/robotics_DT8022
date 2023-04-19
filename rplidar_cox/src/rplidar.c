#include "rplidar.h"

// global circular buffer for storing messages
char messages[BUFFER_BYTES];
volatile int message_count = 0; // Global var to count number of messages
volatile int message_index = 0; // Global var to keep track of message index
volatile int cox_called_flag = 0;

// mutex for the circular buffer
pthread_mutex_t buff_mutex = PTHREAD_MUTEX_INITIALIZER;

void *flush_buffer(void *arg)
{
    server_rp *server = (server_rp *)arg;

    while (1)
    {
        char header[MESSAGE_SIZE] = {0};
        char buffer[MESSAGE_SIZE] = {0};
        while (read(server->new_socket, header, sizeof(header)) == MESSAGE_SIZE)
        {
            if ((__u_char)header[0] == 0xA5 && cox_called_flag == DISABLED)
            {
                if (read(server->new_socket, buffer, sizeof(buffer)) == MESSAGE_SIZE)
                {
                    if (buffer[0] >> 2 != 0)
                    {
                        // add message to circular buffer
                        pthread_mutex_lock(&buff_mutex);
                        memcpy(&messages[message_index], buffer, MESSAGE_SIZE);
                        message_index = (message_index + MESSAGE_SIZE) % BUFFER_BYTES;
                        if (message_count < BUFFER_SIZE)
                        {
                            message_count++;
                        }
                        pthread_mutex_unlock(&buff_mutex);
                    }
                }
            }
        }
    }
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

/*
int main()
{
    server_rp server;
    pthread_t thread;
    init(&server);

    int create_thread_status = pthread_create(&thread, NULL, flush_buffer, &server);

    if (create_thread_status != 0) {
        printf("Error creating thread: %s\n", strerror(create_thread_status));
        exit(EXIT_FAILURE);
    }
    sleep(3); // sleep for 3 seconds

    //sem_init(&sem_rdy_read, 0, 0);

    int count = 1;
    while (1) {
        // sem_wait(&sem_rdy_read);
        sleep(1);
        read_data_test(&count);
        printf("Count: %d\n", count);
    }
    sem_destroy(&sem_rdy_read);
    terminate(&server);
    printf("EXITING\n");
    return 0;
}*/