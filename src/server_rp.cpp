#include "server_rp.h"

void start()
{
    int status, sockkfd;
    struct sockaddr_in serv_addr;
    char startmsg[2];
    startmsg[1] = char(0);
    startmsg[0] = char(16);

    sockkfd = socket(AF_INET, SOCK_STREAM, 0);

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(COMMAND_PORT);
    inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);

    if ((status = connect(sockkfd, (struct sockaddr *)&serv_addr,
                          sizeof(serv_addr))) < 0)
    {
        printf("\nConnection Failed \n");
        return;
    }
    printf("Start sent!\n");
    send(sockkfd, &startmsg, 2, 0);
    close(sockkfd);
}

void stop()
{
    int status, sockkfd;
    struct sockaddr_in serv_addr;
    char startmsg[2];
    startmsg[1] = char(0);
    startmsg[0] = char(32);

    sockkfd = socket(AF_INET, SOCK_STREAM, 0);

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(COMMAND_PORT);
    inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);

    if ((status = connect(sockkfd, (struct sockaddr *)&serv_addr,
                          sizeof(serv_addr))) < 0)
    {
        printf("\nConnection Failed \n");
        return;
    }
    printf("Stop sent!\n");
    send(sockkfd, &startmsg, 2, 0);
    close(sockkfd);
}

void init(server_rp *server)
{
    if ((server->socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    if (setsockopt(server->socket_fd, SOL_SOCKET,
                   SO_REUSEADDR, &server->opt,
                   sizeof(server->address)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    server->address.sin_family = AF_INET;
    server->address.sin_addr.s_addr = INADDR_ANY;
    server->address.sin_port = htons(DATA_PORT);

    if (bind(server->socket_fd, (struct sockaddr *)&server->address,
             sizeof(server->address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    printf("bind sucessfull\n");
    if (listen(server->socket_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    printf("listen sucessfull\n");
    if ((server->new_socket = accept(server->socket_fd, (struct sockaddr *)&server->address,
                                     (socklen_t *)&server->addrlen)) < 0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    printf("accepted\n");
    printf("Connection successfull\n");
}

void terminate(server_rp *server)
{
    stop();
    sleep(1);
    // closing the connected socket
    close(server->new_socket);
    close(server->socket_fd);
    printf("Connection terminated.\n");
}

void read_lidar(server_rp *server)
{
    unsigned char header[5] = {0};
    unsigned char data[4069] = {0};
    int msg_length;
    int start, quality, angle, distance;

    int nr_ch = read(server->new_socket, &header, 5);
    if (nr_ch != 5)
        return;

    if (header[0] == 0xA5)
    {
        msg_length = ((header[2] << 16) + (header[3] << 8) + (header[4]));
        read(server->new_socket, &data, msg_length);
        start = data[0] & 1;
        quality = data[0] >> 2;
        angle = ((data[1] >> 1) + (data[2] << 8)) >> 7;
        distance = ((data[3]) + (data[4] << 8)) >> 2;
        printf("length: %d Start: %d, Quality: %d, Angle: %d, Dist: %d\n", msg_length, start, quality, angle, distance);
    }
}
