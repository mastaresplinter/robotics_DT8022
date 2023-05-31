#ifdef __cplusplus
extern "C"
{
#endif

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

    typedef struct server_rplidar
    {
        struct sockaddr_in address;
        int socket_fd;
        int new_socket;
        int opt;
        int addrlen;
    } server_rp;

    void stop();
    void start();
    void init(server_rp *server);
    void terminate(server_rp *server);
    void read_lidar(server_rp *server);

#endif

#ifdef __cplusplus
}
#endif