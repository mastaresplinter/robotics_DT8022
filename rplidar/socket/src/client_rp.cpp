#include "client_rp.h"

void crp::client_rp::init() {
    struct sockaddr_in serv_addr;
    
    client_fd = socket(AF_INET,SOCK_STREAM,0);
    
    serv_addr.sin_family=AF_INET;
    serv_addr.sin_port = htons(COMMAND_PORT);
    inet_pton(AF_INET, IP_ADDRESS, &serv_addr.sin_addr);
    
    if ((connect(client_fd, (struct sockaddr*)&serv_addr,
                sizeof(serv_addr)))
        < 0) {
        perror("Connection Failed \n");
        exit(EXIT_FAILURE);
    }
    printf("Client initialized");
}

void crp::client_rp::start(){
    char startmsg = START_MSG;
    send(client_fd, &startmsg, 1, 0);
}
void crp::client_rp::stop(){
    char stopmsg = STOP_MSG;
    send(client_fd, &stopmsg, 1, 0);
}

void crp::client_rp::terminate()
{
    if(close(client_fd) < 0){
        perror("Closing connection failed.\n");
        exit(EXIT_FAILURE);
    }
}
