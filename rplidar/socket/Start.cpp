#include <iostream>
#include <arpa/inet.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define PORT 9887
using namespace std;

int main()
{
    int status,sockkfd;
    struct sockaddr_in serv_addr;
    char startmsg[2];
    startmsg[1] = char(0);
    startmsg[0] = char(16);
    
    sockkfd = socket(AF_INET,SOCK_STREAM,0);
    
    serv_addr.sin_family=AF_INET;
    serv_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
    
    if ((status
         = connect(sockkfd, (struct sockaddr*)&serv_addr,
                   sizeof(serv_addr)))
        < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }
    printf("\n Connection Succesful");

    send(sockkfd, &startmsg, 2, 0);
    return 0;
}