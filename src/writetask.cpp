#include <eigen/Eigen/Dense>
#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include "stop_vars.h"
#include "global_vars.h"
#include "writetask.h"

using namespace Eigen;

#define ODOPATH "textfiles/odo.txt"
#define COXPATH "textfiles/cox.txt"
#define KALPATH "textfiles/kalman.txt"

typedef struct
{
    double Timestamp;
    Matrix<double, 3, 1> positions;
    Matrix<double, 3, 3> uncertainty;

} Data;

pthread_mutex_t filewrite_mutex = PTHREAD_MUTEX_INITIALIZER;

void WriteDataToFile(Data *data, int choice)
{
    FILE *file;
    switch (choice)
    {
    case 1:
        file = fopen(ODOPATH, "a");
        break;
    case 2:
        file = fopen(COXPATH, "a");
        break;
    case 3:
        file = fopen(KALPATH, "a");
        break;
    default:
        return;
    }

    fprintf(file, "%f %f %f %f %f %f %f %f %f %f %f %f %f\n",
            data->Timestamp, data->positions(0, 0), data->positions(1, 0), data->positions(2, 0),
            data->uncertainty(0, 0), data->uncertainty(0, 1), data->uncertainty(0, 2),
            data->uncertainty(1, 0), data->uncertainty(1, 1), data->uncertainty(1, 2),
            data->uncertainty(2, 0), data->uncertainty(2, 1), data->uncertainty(2, 2));

    fclose(file);
}

void *WriteFilesThread(void *arg)
{
    FILE *file;
    file = fopen(ODOPATH, "w");
    if (!file)
    {
        // handle the error
        printf("something went wrong opening odo.txt: %s", strerror(errno));
        exit(1);
    }
    fclose(file);
    file = fopen(COXPATH, "w");
    if (!file)
    {
        // handle the error
        printf("something went wrong opening cox.txt: %s", strerror(errno));
        exit(1);
    }
    fclose(file);
    file = fopen(KALPATH, "w");
    if (!file)
    {
        // handle the error
        printf("something went wrong opening kalman.txt: %s", strerror(errno));
        exit(1);
    }
    fclose(file);

    Data odo_data, cox_data, kal_data;

    static struct timeval initial_timestamp = {0, 0};
    struct timeval current_timestamp;
    double elapsed_time;
    while (global_stop_msg != ACTIVE)
    {
        gettimeofday(&current_timestamp, NULL);

        // Set initial time stamp
        if (initial_timestamp.tv_sec == 0 && initial_timestamp.tv_usec == 0)
        {
            initial_timestamp = current_timestamp;
        }

        // Beräkna elapsed time
        elapsed_time = (double)((current_timestamp.tv_sec - initial_timestamp.tv_sec) * 1000000 + (current_timestamp.tv_usec - initial_timestamp.tv_usec)) / 1000000;

        odo_data.Timestamp = elapsed_time;
        cox_data.Timestamp = elapsed_time;
        kal_data.Timestamp = elapsed_time;

        // odo_data.positions << 1, 2, 3;
        // odo_data.uncertainty << 1, 2, 3,
        //                 4, 5, 6,
        //                 7, 8, 9;

        // cox_data.positions << 1, 2, 3;
        // cox_data.uncertainty << 1, 2, 3,
        //                 4, 5, 6,
        //                 7, 8, 9;

        // kal_data.positions << 1, 2, 3;
        // kal_data.uncertainty << 1, 2, 3,
        //                 4, 5, 6,
        //                 7, 8, 9;
        // pthread_mutex_lock(&filewrite_mutex)
        odo_data.positions = global_pos;
        odo_data.uncertainty = global_Cxya;

        cox_data.positions = global_Poscox;
        cox_data.uncertainty = global_Ccox;

        kal_data.positions = global_Poskal;
        kal_data.uncertainty = global_Ckal;
        // pthread_mutex_unlock(&filewrite_mutex);

        WriteDataToFile(&odo_data, 1);
        WriteDataToFile(&cox_data, 2);
        WriteDataToFile(&kal_data, 3);

        usleep(100000); // Sleep for 1s
    }

    return NULL;
}

// int main(void)
// {
//     global_Cxya << 1, 2, 3,
//                     4, 5, 6,
//                     7, 8, 9;
//     global_Ccox << 1, 2, 3,
//                     4, 5, 6,
//                     7, 8, 9;
//     global_Ckal << 1, 2, 3,
//                     4, 5, 6,
//                     7, 8, 9;

//     global_Posxya << 1, 2, 3;
//     global_Poscox << 1, 2, 3;
//     global_Poskal << 1, 2, 3;

//     pthread_t thread;
//     pthread_create(&thread, NULL, WriteFilesThread, NULL);
//     pthread_join(thread, NULL);
//     return 0;
// }