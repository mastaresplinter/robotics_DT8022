#include "server_rp.h"

void srp::server_rp::init() {
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    
    if (setsockopt(server_fd, SOL_SOCKET,
                SO_REUSEADDR, &opt,
                sizeof(opt))) 
                {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(DATA_PORT);

    if (bind(server_fd, (struct sockaddr*)&address,
        sizeof(address))
        < 0) 
        {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    printf("bind sucessfull\n");
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    printf("listen sucessfull\n");
    if ((new_socket
        = accept(server_fd, (struct sockaddr*)&address,
                (socklen_t*)&addrlen))
        < 0) 
        {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    printf("accepted\n");
    printf("Connection successfull\n");
    //terminate();
}
void srp::server_rp::terminate() {
    // closing the connected socket
    close(new_socket);
    // closing the listening socket
    shutdown(server_fd, SHUT_RDWR);
    printf("Connection terminated.");
}
void srp::server_rp::read_lidar(){
    char header[5] = { 0 };
    char data[4069] = { 0 };
    int msg_length;
    int start, quality, angle, distance;
    
 
    int nr_ch = read(new_socket, header, 5);
    if(nr_ch != 5)
        return;

    if((int)header[0] == 0xA5)
    {
        msg_length = ((header[2]<<16) + (header[3]<<8) + (header[4]));
        read(new_socket, data, msg_length);
        start = data[0] & 1;
        quality = data[0]>>2;
        angle = ((data[1]>>1) + (data[2]<<8))>>7;
        distance = ((data[3]) + (data[4]<<8))>>2;
        printf("length: %d Start: %d, Quality: %d, Angle: %d, Dist: %d\n", msg_length, start, quality, angle, distance);
    }
}

