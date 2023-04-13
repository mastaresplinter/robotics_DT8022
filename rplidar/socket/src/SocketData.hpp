#ifndef SOCKETDATA_H
#define SOCKETDATA_H

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h> 
#include <unistd.h>
#include <time.h>
#include <thread>         // std::thread

#define IP_ADDRESS "127.0.0.1"
#define DATA_PORT 9888
#define COMMAND_PORT 9887
#define START_MSG 16
#define STOP_MSG 32

#endif