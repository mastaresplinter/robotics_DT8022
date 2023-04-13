#include "server_rp.h"
// #include "lidar_message.h"
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>

#define BUFFER_SIZE 200
#define MESSAGE_SIZE 5
#define BUFFER_BYTES (BUFFER_SIZE * MESSAGE_SIZE)

// global circular buffer for storing messages
char messages[BUFFER_BYTES];
// lidar_message_t messages[BUFFER_SIZE];
int message_count = 0;
int message_index = 0;
sem_t sem_rdy_read;

// mutex for accessing the circular buffer
pthread_mutex_t buff_mutex = PTHREAD_MUTEX_INITIALIZER;


void *myThreadFun(void *vargp)
{
    server_rp *server=(server_rp *) vargp; 
    sleep(1);
    init(&server);   
    volatile time_t start_t = clock();
    volatile time_t prev_t = start_t;
    while ((prev_t - start_t)/CLOCKS_PER_SEC < 5)
    {
        prev_t = clock();
        read_lidar(&server);
    }
    terminate(&server);
    return NULL;
}
void *flush_buffer(void *arg)
{   
    server_rp *server = (server_rp *) arg;
    int count = 0;
    while (1) {
        //sleep(3); // sleep for 3 seconds
        char header[MESSAGE_SIZE] = {0};
        char buffer[MESSAGE_SIZE] = {0};
        count = 0;
        while (read(server->new_socket, header, sizeof(header)) == MESSAGE_SIZE) 
        {
            if((__u_char)header[0] == 0xA5)
            {
                if(read(server->new_socket, buffer, sizeof(buffer)) == MESSAGE_SIZE) 
                {       
                        if(buffer[0]>>2 != 0)
                        {
                            // add message to circular buffer
                            pthread_mutex_lock(&buff_mutex);
                            // memcpy(&messages[message_index].data[0], buffer, MESSAGE_SIZE);
                            // message_index = (message_index + 1) % BUFFER_SIZE;
                            memcpy(&messages[message_index], buffer, MESSAGE_SIZE);
                            message_index = (message_index + MESSAGE_SIZE) % BUFFER_BYTES;
                            if (message_count < BUFFER_SIZE) {
                                message_count++;
                            }
                            pthread_mutex_unlock(&buff_mutex);
                        }
                }
            }
            count++;
        }
        //sem_post(&sem_rdy_read);
    }
}

void read_data_test(int *count){
    int start, quality, angle, distance;
    pthread_mutex_lock(&buff_mutex);
    int i, j;
    // for (i = 0, j = message_index; i < message_count; i++, j = (j + MESSAGE_SIZE) % BUFFER_BYTES) {
        // start = messages[j].data[0] & 1;
        // quality = messages[j].data[0]>>2;
        // angle = ((messages[j].data[1]>>1) + (messages[j].data[2]<<8))>>7;
        // distance = ((messages[j].data[3]) + (messages[j].data[4]<<8))>>2;
        // printf("Start: %d, Quality: %d, Angle: %d, Dist: %d\n", start, quality, angle, distance);
    // }
    for (i = 0, j = message_index; i < message_count; i++, j = (j + MESSAGE_SIZE) % BUFFER_BYTES) {
        start = messages[j] & 1;
        quality = messages[j]>>2;
        angle = ((messages[j+1]>>1) + (messages[j+2]<<8))>>7;
        distance = ((messages[j+3]) + (messages[j+4]<<8))>>2;
        printf("Start: %d, Quality: %d, Angle: %d, Dist: %d\n", start, quality, angle, distance);
    }
    pthread_mutex_unlock(&buff_mutex);
    *count = *count + 1;
}

int main()
{
    server_rp server;
    pthread_t thread;
    init(&server);
    //printf("Socket: %d\n", server.socket_fd);
    
    int create_thread_status = pthread_create(&thread, NULL, flush_buffer, &server);
    
    if (create_thread_status != 0) {
        printf("Error creating thread: %s\n", strerror(create_thread_status));
        exit(EXIT_FAILURE);
    }
    sleep(3); // sleep for 3 seconds

    sem_init(&sem_rdy_read, 0, 0);
    
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
}