#include "server_rp.h"
#include "rplidar.h"
#include "task1.h"
#include "task2.h"
#include "task3.h"
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include "stop_vars.h"
#include <errno.h>
#include "writetask.h"

using namespace std;

pthread_t init_thread;
pthread_t flush_thread;
pthread_t task1_thread;
pthread_t task2_thread;
pthread_t task3_thread;
pthread_t filewrite_thread;
server_rp server;

void signal_callback_handler(int signum)
{
    // Terminate program
    global_stop_msg = ACTIVE;
    terminate(&server);
    cout << "Caught signal " << signum << endl;
    sleep(1);
    pthread_join(filewrite_thread, NULL);
    printf("filewrite_thread finish!\n");

    pthread_join(flush_thread, NULL);
    printf("flush_thread finish!\n");

    pthread_join(task1_thread, NULL);
    printf("task1_thread finish!\n");

    pthread_join(task3_thread, NULL);
    printf("task3_thread finish!\n");

    printf("EXITING\n");

    exit(signum);
}

void *init_lidar(void *arg)
{
    server_rp *server = (server_rp *)arg;
    init(server);
    return NULL;
}

int main()
{
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = signal_callback_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    int create_thread_status = pthread_create(&init_thread, NULL, init_lidar, &server);

    if (create_thread_status != 0)
    {
        printf("Error creating init_thread: %s\n", strerror(create_thread_status));
        exit(EXIT_FAILURE);
    }
    printf("init_thread start!\n");
    sleep(1);
    start();

    pthread_join(init_thread, NULL);
    printf("init_thread finish!\n");

    create_thread_status = pthread_create(&flush_thread, NULL, flush_buffer, &server);

    if (create_thread_status != 0)
    {
        printf("Error creating flush_thread: %s\n", strerror(create_thread_status));
        exit(EXIT_FAILURE);
    }
    printf("flush_thread started!\n");

    sleep(1); // sleep for 1 seconds

    create_thread_status = pthread_create(&task2_thread, NULL, Task2Thread, NULL);
    if (create_thread_status != 0)
    {
        printf("Error creating task1_thread: %s\n", strerror(create_thread_status));
        exit(EXIT_FAILURE);
    }
    printf("task2_thread started!\n");

    sleep(1); // sleep for 1 seconds

    create_thread_status = pthread_create(&task3_thread, NULL, Task3Thread, NULL);
    if (create_thread_status != 0)
    {
        printf("Error creating task1_thread: %s\n", strerror(create_thread_status));
        exit(EXIT_FAILURE);
    }
    printf("task3_thread started!\n");

    sleep(1); // sleep for 1 seconds

    create_thread_status = pthread_create(&task1_thread, NULL, Task1Thread, NULL);
    if (create_thread_status != 0)
    {
        printf("Error creating task1_thread: %s\n", strerror(create_thread_status));
        exit(EXIT_FAILURE);
    }
    printf("task1_thread started!\n");

    sleep(1); // sleep for 1 seconds

    create_thread_status = pthread_create(&filewrite_thread, NULL, WriteFilesThread, NULL);
    if (create_thread_status != 0)
    {
        printf("Error creating filewrite_thread: %s\n", strerror(create_thread_status));
        exit(EXIT_FAILURE);
    }
    printf("filewrite_thread started!\n");

    sleep(1); // sleep for 1 seconds

    pause();

    return EXIT_SUCCESS;
}
