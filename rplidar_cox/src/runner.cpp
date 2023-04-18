#include "server_rp.h"
#include "rplidar.h"
#include "cox.hpp"

int main()
{

    server_rp server;
    pthread_t thread;
    init(&server);

    int create_thread_status = pthread_create(&thread, NULL, flush_buffer, &server);

    if (create_thread_status != 0)
    {
        printf("Error creating thread: %s\n", strerror(create_thread_status));
        exit(EXIT_FAILURE);
    }
    sleep(3); // sleep for 3 seconds

    int count = 0;
    while (1)
    {
        sleep(1);
        // read_data_test(&count);
        CoxAlgo();
        count++;
        printf("Count: %d\n", count);
    }

    pthread_join(thread, NULL);
    terminate(&server);
    printf("EXITING\n");
    return 0;
}
