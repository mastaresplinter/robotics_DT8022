#include "server_rp.h"
#include "client_rp.h"
#include <time.h>
#include <thread>

using namespace std;
using namespace srp;
using namespace crp;

int main(int argc, char const* argv[])
{
    // int server_fd, new_socket, nr_ch;
    // struct sockaddr_in address;
    // int addrlen = sizeof(address);
    // int opt = 1;
    // char header[5] = { 0 };
    // char data[4069] = { 0 };
    // int msg_length;
    // int start, quality, angle, distance;
    
 
    // int check = 0;
    // while(check < 500)
    // {
    //     nr_ch = read(new_socket, header, 5);
    //     if(nr_ch != 5)
    //         break;
    //     if((int)header[0] == 0xA5)
    //     {
    //         msg_length = ((header[2]<<16) + (header[3]<<8) + (header[4]));
    //         read(new_socket, data, msg_length);
    //         start = data[0] & 1;
    //         quality = data[0]>>2;
    //         angle = ((data[1]>>1) + (data[2]<<8))>>7;
    //         distance = ((data[3]) + (data[4]<<8))>>2;
    //         printf("Start: %d, Quality: %d, Angle: %d, Dist: %d\n", start, quality, angle, distance);
    //     }
    //     check++;
    // }
    
    server_rp server;

    std::thread server_t([&server]() {
        server.init();        
        volatile time_t start_t = clock();
        volatile time_t prev_t;
        while ((prev_t - start_t)/CLOCKS_PER_SEC < 5)
        {
            prev_t = clock();
            server.read_lidar();
        }
    });

    server_t.join();
    server.terminate();
    printf("EXITING\n");
    return 0;
}