#ifndef SERVER_RP_H
#define SERVER_RP_H

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h> 
#include <unistd.h>

#define IP_ADDRESS "127.0.0.1"
#define DATA_PORT 9888
#define COMMAND_PORT 9887
#define START_MSG 16
#define STOP_MSG 32

namespace srp
{
    class server_rp{
        private:
            struct sockaddr_in address;
            int server_fd;
            int new_socket;
            int opt;
            int addrlen;
        public:
            void init();
            void terminate();
            void read_lidar();
    };
}

#endif