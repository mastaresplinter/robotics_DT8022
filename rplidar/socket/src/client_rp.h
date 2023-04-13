#ifndef CLIENT_RP_H
#define CLIENT_RP_H

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


namespace crp{
    class client_rp{
        private:
        int client_fd;
        struct sockaddr_in serv_addr;

        public:
        void init();
        void start();
        void stop();
        void terminate();
    };
}


#endif