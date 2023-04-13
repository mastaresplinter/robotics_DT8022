#include "server_rp.h"
#include <time.h>
#include <thread>

using namespace std;
using namespace _rpi;


class client_rp {
    public:
    int client_fd;
    struct sockaddr_in serv_addr;

    int init() {
        struct sockaddr_in serv_addr;
        
        client_fd = socket(AF_INET,SOCK_STREAM,0);
        
        serv_addr.sin_family=AF_INET;
        serv_addr.sin_port = htons(COMMAND_PORT);
        inet_pton(AF_INET, IP_ADDRESS, &serv_addr.sin_addr);
        
        if ((connect(client_fd, (struct sockaddr*)&serv_addr,
                    sizeof(serv_addr)))
            < 0) {
            printf("\nConnection Failed \n");
            exit(EXIT_FAILURE);
        }
        printf("Client initialized");
        return 0;
    }

    void start(){
        char startmsg = START_MSG;
        send(client_fd, &startmsg, 1, 0);
    }
    void stop(){
        char stopmsg = STOP_MSG;
        send(client_fd, &stopmsg, 1, 0);
    }
};

int main(int argc, char const* argv[])
{
    time_t start_t = clock();
    time_t prev_t;
    
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
    client_rp client;

    std::thread server_t (server.init());
    std::thread client_t (client.init());

    client.start();

    while ((prev_t - start_t)/CLOCKS_PER_SEC < 10)
    {
        prev_t = clock();
    }
    
    client.stop();

    server_t.join();
    client_t.join();

    return 0;
}