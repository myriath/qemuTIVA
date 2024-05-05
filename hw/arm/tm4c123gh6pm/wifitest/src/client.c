#include "main.h"

int client_connect(char *ip, int port, void (*handle)(void))
{
    printf("Connecting to %s:%d\n", ip, port);
    int fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
        return ERR_SOCK_FAILED;
    }
    
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = port,
        .sin_addr.s_addr = inet_addr(ip)
    };
    memset(&(address.sin_zero), 0, 8);
    int status = connect(fd, (struct sockaddr *) &address, sizeof(address));
    if (status < 0) {
        return ERR_CONNECT;
    }

    handle();

    close(fd);
    return SUCCESS;
}

void handler(void)
{
    printf("Connected\n");
}

int main(int argc, char **argv)
{
    if (argc != 3) exit(EXIT_FAILURE);

    char *ip = argv[1];
    if (strcmp(ip, "localhost") == 0) {
        ip = "127.0.0.1";
    }
    int port = atoi(argv[2]);

    switch (client_connect(ip, port, handler)) {
        case SUCCESS:
            exit(EXIT_SUCCESS);
        case ERR_SOCK_FAILED:
            perror("Failed to create socket");
            break;
        case ERR_CONNECT:
            perror("Failed to connect");
            break;
        default:
            perror("Error");
    }
    exit(EXIT_FAILURE);
}